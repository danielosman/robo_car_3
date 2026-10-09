#!/usr/bin/env python3
"""Worst-case stack use of picoA_app, from GCC's per-function call graphs.

Build a copy of the firmware with the call graphs (the normal build/ stays as is):

    cmake -S . -B build/stack -G Ninja -DPICO_BOARD=pico2_w \
        -DPICO_PLATFORM=rp2350-arm-s -DCMAKE_BUILD_TYPE=Release \
        "-DCMAKE_C_FLAGS=-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp \
         -mcmse -mfloat-abi=softfp -ftls-model=local-exec -fcallgraph-info=su,da"
    ninja -C build/stack picoA_app
    tools/stack_depth.py build/stack/picoA

It prints the deepest call chain from main and from each interrupt handler, then
the core-0 worst case: main + the deepest lowest-priority handler + the deepest
default-priority handler + two exception frames (handlers of one priority don't
nest). Calls through function pointers aren't in GCC's graph: INDIRECT adds the
ones that matter (printf's output, lwIP's callbacks and timers); add new ones
there. libc/libgcc helpers have no graph and count as 0 (small leaf functions).
Cutting lwIP's cycles makes its number depend on where the cut falls (0.75-1.3 KB
on 9 Oct 2026); the search order is fixed so the result repeats (1.2 KB).
"""
import os, re, sys

INDIRECT = [  # caller > callee, as file:function or function
    'vprintf>__wrap_vprintf',
    'printf.c:_out_fct>stdio.c:stdio_buffered_printer',
    'stdio.c:stdio_out_chars_crlf>stdio_usb.c:stdio_usb_out_chars',
    'stdio.c:stdio_out_chars_crlf>wifi_console.c:out_chars',
    'stdio.c:stdio_out_chars_no_crlf>stdio_usb.c:stdio_usb_out_chars',
    'stdio.c:stdio_out_chars_no_crlf>wifi_console.c:out_chars',
    'ip4_output_if_src>etharp_output',
    'etharp_output>ethernet_output',
    'ethernet_output>cyw43_lwip.c:cyw43_netif_output',
    'cyw43_cb_process_ethernet>ethernet_input',
    'async_context_threadsafe_background.c:process_under_lock>cyw43_driver.c:cyw43_do_poll',
    'async_context_threadsafe_background.c:process_under_lock>lwip_nosys.c:lwip_timeout_reached',
    'async_context_threadsafe_background.c:process_under_lock>cyw43_driver.c:cyw43_sleep_timeout_reached',
    'tcp_input>wifi_console.c:on_received',
    'tcp_input>wifi_console.c:on_sent',
    'tcp_input>wifi_console.c:on_error',
    'tcp_input>wifi_console.c:on_connected',
    'tcp_slowtmr>wifi_console.c:on_error',
    'udp_input>wifi_console.c:on_announcement',
    'sys_check_timeouts>tcp_tmr',
    'sys_check_timeouts>dhcp_fine_tmr',
    'sys_check_timeouts>dhcp_coarse_tmr',
    'sys_check_timeouts>etharp_tmr',
    'sys_check_timeouts>ip_reass_tmr',
]
LOWEST_PRIORITY = [  # the SDK's low-priority workers (USB stdio, cyw43 + lwIP)
    'stdio_usb.c:low_priority_worker_irq',
    'async_context_threadsafe_background.c:low_priority_irq_handler',
]
DEFAULT_PRIORITY = [
    'link.c:on_uart_irq', 'camera.c:frame_done', 'dcd_rp2040.c:dcd_rp2040_irq',
    'timer.c:hardware_alarm_irq_handler', 'time.c:alarm_pool_irq_handler',
    'cyw43_driver.c:cyw43_gpio_irq_handler', 'gpio.c:gpio_default_irq_handler',
]
EXCEPTION_FRAME = 104  # bytes pushed per interrupt with the FPU context

nodes, edges, titles = {}, {}, set()  # title -> bytes; title -> callees; all
node_re = re.compile(r'node: \{ title: "([^"]+)" label: "([^"]*)"')
edge_re = re.compile(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"')
for d, _, files in os.walk(sys.argv[1]):
    for f in files:
        if f.endswith('.ci'):
            for line in open(os.path.join(d, f)):
                if m := node_re.search(line):
                    titles.add(m.group(1))
                    if s := re.search(r'\\n(\d+) bytes', m.group(2)):
                        nodes[m.group(1)] = int(s.group(1))
                elif m := edge_re.search(line):
                    edges.setdefault(m.group(1), set()).add(m.group(2))


def find(name):
    hits = [t for t in titles if t == name or t.endswith('/' + name) or t.endswith(':' + name)]
    if len(hits) != 1:
        sys.exit(f'{name}: {len(hits)} matches')
    return hits[0]


sys.setrecursionlimit(20000)
for pair in INDIRECT:
    caller, callee = pair.split('>')
    edges.setdefault(find(caller), set()).add(find(callee))


memo, active = {}, set()


def deepest(t):
    """(bytes, chain) of the deepest call chain from t. A cycle (lwIP sending
    while receiving) is cut where it closes, so it counts once."""
    if t in memo:
        return memo[t]
    if t in active:
        return 0, ()
    active.add(t)
    best = max((deepest(c) for c in sorted(edges.get(t, ()))), default=(0, ()))
    active.discard(t)
    memo[t] = nodes.get(t, 0) + best[0], (f'{t.split(":")[-1]} {nodes.get(t, 0)}',) + best[1]
    return memo[t]


def show(name):
    total, chain = deepest(find(name))
    print(f'{total:5} B  {name}\n         ' + ' > '.join(chain))
    return total


main = show('main')
low = max(show(h) for h in LOWEST_PRIORITY)
high = max(show(h) for h in DEFAULT_PRIORITY)
print(f'core 0 worst case: {main} + {low} + {high} + 2 x {EXCEPTION_FRAME} = '
      f'{main + low + high + 2 * EXCEPTION_FRAME} B')
