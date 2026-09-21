/**
 * Pico C SDK platform layer for the VL53L8CX ULD API (SPI).
 *
 * Ported from ST's STSW-IMG040 SPI example (STM32 HAL) to the Raspberry Pi
 * Pico 2 W (RP2350). Implements the transport functions the ULD API calls.
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>
#include "hardware/spi.h"

/* PicoA PCB: Pololu VL53L8CX #3419, SPI0. LPn is pulled high on carrier. */
#define VL53L8CX_SPI_PORT       spi0
#define VL53L8CX_SPI_BAUDRATE   (2500u * 1000u)  /* 2.5 MHz (datasheet max 3 MHz) */
#define VL53L8CX_PIN_SCK        18   /* SCL/MCLK -> SPI0 SCK */
#define VL53L8CX_PIN_MOSI       19   /* SDA/MOSI -> SPI0 TX  */
#define VL53L8CX_PIN_MISO       16   /* MISO     -> SPI0 RX  */
#define VL53L8CX_PIN_CS         17   /* CS  (software-driven, active low) */
#define VL53L8CX_PIN_INT        20   /* INT input only; NOT the old project's LPn */

/**
 * @brief Platform descriptor. The ULD API only uses 'address' (I2C); for SPI it
 * is unused but kept for API compatibility.
 */
typedef struct
{
	uint16_t address;
} VL53L8CX_Platform;

/*
 * @brief Number of targets reported per zone (1..4). 1 = report only the
 * closest/strongest single target per zone (selected via target order).
 */
#define 	VL53L8CX_NB_TARGET_PER_ZONE		4U

/*
 * @brief Use firmware data format directly (skip fw<->user conversion).
 */
// #define 	VL53L8CX_USE_RAW_FORMAT

/*
 * @brief Output enable/disable macros (define to drop an output and cut bus traffic).
 */
// #define VL53L8CX_DISABLE_AMBIENT_PER_SPAD
// #define VL53L8CX_DISABLE_NB_SPADS_ENABLED
// #define VL53L8CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L8CX_DISABLE_SIGNAL_PER_SPAD
// #define VL53L8CX_DISABLE_RANGE_SIGMA_MM
// #define VL53L8CX_DISABLE_DISTANCE_MM
// #define VL53L8CX_DISABLE_REFLECTANCE_PERCENT
// #define VL53L8CX_DISABLE_TARGET_STATUS
#define VL53L8CX_DISABLE_MOTION_INDICATOR

/**
 * @brief Initialise SPI0 and the CS/INT GPIOs for the VL53L8CX.
 * Call once before any ULD API function. Returns 0 on success.
 */
uint8_t vl53l8cx_pico_init(VL53L8CX_Platform *p_platform);

/* ---- ULD transport functions (mandatory) --------------------------------- */

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value);

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value);

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform);

void VL53L8CX_SwapBuffer(
		uint8_t *buffer,
		uint16_t size);

uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs);

#endif	// _PLATFORM_H_
