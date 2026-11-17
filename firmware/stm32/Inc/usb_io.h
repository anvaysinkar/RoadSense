#ifndef USB_IO_H
#define USB_IO_H

#include <stdint.h>

void usb_io_write_line(const char* s);         /* queue one line + "\r\n"; dropped if the buffer is full */
void usb_io_pump(void);                        /* call often from the main loop */
void usb_io_rx(const uint8_t* buf, uint32_t len);   /* call from CDC_Receive_FS (interrupt) */
int  usb_io_getc(void);                        /* next received key, -1 if none */
uint32_t usb_io_dropped(void);

#endif
