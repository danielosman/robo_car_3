/**
 * Pico C SDK platform layer for the VL53L8CX ULD API (SPI).
 * Ported from ST STSW-IMG040 SPI example to the Raspberry Pi Pico 2 W.
 */

#include "platform.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* SPI framing: the 16-bit register index is sent MSB-first, with the top bit
 * marking direction (write = 1, read = 0). CS is held low for the whole
 * address+data transfer. */
#define SPI_WRITE_MASK(x) (uint16_t)((x) | 0x8000)
#define SPI_READ_MASK(x)  (uint16_t)((x) & ~0x8000)

/* Re-issue the start address every CHUNK bytes (matches ST reference). Kept
 * modest so we never need a large stack buffer. */
#define VL53L8CX_COMMS_CHUNK_SIZE 1024u

static inline void cs_low(void)  { gpio_put(VL53L8CX_PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(VL53L8CX_PIN_CS, 1); }

uint8_t vl53l8cx_pico_init(VL53L8CX_Platform *p_platform)
{
	p_platform->address = 0x52;  /* unused in SPI mode, kept for API */

	/* SPI peripheral: mode 3 (CPOL=1, CPHA=1), 8-bit, MSB first. */
	spi_init(VL53L8CX_SPI_PORT, VL53L8CX_SPI_BAUDRATE);
	spi_set_format(VL53L8CX_SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
	gpio_set_function(VL53L8CX_PIN_SCK,  GPIO_FUNC_SPI);
	gpio_set_function(VL53L8CX_PIN_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(VL53L8CX_PIN_MISO, GPIO_FUNC_SPI);

	/* CS as software-driven GPIO, idle high. */
	gpio_init(VL53L8CX_PIN_CS);
	gpio_put(VL53L8CX_PIN_CS, 1);
	gpio_set_dir(VL53L8CX_PIN_CS, GPIO_OUT);

	/* LPn and SPI_I2C_N are held high by the carrier; no Pico GPIO attached. */

	/* INT data-ready input (carrier pulls it up; active low). */
	gpio_init(VL53L8CX_PIN_INT);
	gpio_set_dir(VL53L8CX_PIN_INT, GPIO_IN);

	return 0;
}

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	return VL53L8CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	return VL53L8CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	(void)p_platform;
	uint32_t position;

	for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE)
	{
		uint32_t data_size = (size - position > VL53L8CX_COMMS_CHUNK_SIZE)
				? VL53L8CX_COMMS_CHUNK_SIZE : (size - position);
		uint16_t reg = SPI_WRITE_MASK(RegisterAdress + position);
		uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

		cs_low();
		spi_write_blocking(VL53L8CX_SPI_PORT, addr, 2);
		spi_write_blocking(VL53L8CX_SPI_PORT, p_values + position, data_size);
		cs_high();
	}
	return 0;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	(void)p_platform;
	uint32_t position;

	for (position = 0; position < size; position += VL53L8CX_COMMS_CHUNK_SIZE)
	{
		uint32_t data_size = (size - position > VL53L8CX_COMMS_CHUNK_SIZE)
				? VL53L8CX_COMMS_CHUNK_SIZE : (size - position);
		uint16_t reg = SPI_READ_MASK(RegisterAdress + position);
		uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

		cs_low();
		spi_write_blocking(VL53L8CX_SPI_PORT, addr, 2);
		spi_read_blocking(VL53L8CX_SPI_PORT, 0x00, p_values + position, data_size);
		cs_high();
	}
	return 0;
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
	(void)p_platform;
	/* No hardware reset available. LPn is NOT a sensor reset even when wired.
	 * A true reset requires a power cycle (UM3109 section 4.2).
	 * ULD initialization does its own software boot; this hook is not called. */
	return 255;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
	uint32_t i, tmp;

	for (i = 0; i < size; i += 4)
	{
		tmp = ((uint32_t)buffer[i] << 24)
			| ((uint32_t)buffer[i + 1] << 16)
			| ((uint32_t)buffer[i + 2] << 8)
			| (buffer[i + 3]);
		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
	(void)p_platform;
	sleep_ms(TimeMs);
	return 0;
}
