/*
 * usb_io.c -- buffered output over USB CDC (virtual COM port) and key input.
 * Lines go into an 8 KB ring; usb_io_pump() hands contiguous chunks to CDC_Transmit_FS
 * and advances only after the previous transfer has finished. While no terminal has
 * the port open the ring fills and new lines are dropped (counted), so the 100 Hz loop
 * never blocks.
 */
#include "usb_io.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

#define TX_N 8192u
#define RX_N 64u
#define CHUNK_MAX 512u

static uint8_t tx[TX_N];
static volatile uint32_t txHead = 0, txTail = 0;
static uint32_t inflight = 0, dropped = 0;

static volatile uint8_t rx[RX_N];
static volatile uint32_t rxHead = 0, rxTail = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint32_t tx_used(void) { return (txHead - txTail) % TX_N; }

void usb_io_write_line(const char* s) {
  uint32_t n = (uint32_t)strlen(s);
  if (tx_used() + n + 2 >= TX_N - 1) { dropped++; return; }   /* whole line or nothing */
  for (uint32_t i = 0; i < n; i++) { tx[txHead] = (uint8_t)s[i]; txHead = (txHead + 1) % TX_N; }
  tx[txHead] = '\r'; txHead = (txHead + 1) % TX_N;
  tx[txHead] = '\n'; txHead = (txHead + 1) % TX_N;
}

void usb_io_pump(void) {
  USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) return;                      /* not enumerated yet */
  if (inflight) {
    if (hcdc->TxState != 0) return;              /* previous transfer still running */
    txTail = (txTail + inflight) % TX_N;
    inflight = 0;
  }
  uint32_t used = tx_used();
  if (!used) return;
  uint32_t contig = (txHead >= txTail) ? used : TX_N - txTail;   /* up to the end of the ring */
  uint32_t n = contig < CHUNK_MAX ? contig : CHUNK_MAX;
  if (n > 1 && (n % 64u) == 0) n--;              /* avoid exact multiples of 64 (no zero-length packet) */
  if (CDC_Transmit_FS(&tx[txTail], (uint16_t)n) == USBD_OK) inflight = n;
}

void usb_io_rx(const uint8_t* buf, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    uint32_t nh = (rxHead + 1) % RX_N;
    if (nh == rxTail) break;
    rx[rxHead] = buf[i]; rxHead = nh;
  }
}

int usb_io_getc(void) {
  if (rxTail == rxHead) return -1;
  int c = rx[rxTail]; rxTail = (rxTail + 1) % RX_N;
  return c;
}

uint32_t usb_io_dropped(void) { return dropped; }
