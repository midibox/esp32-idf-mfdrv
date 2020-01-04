/*
 * Upper MIDI Layer (might be a separate component in future...)
 *
 * =============================================================================
 *
 * MIT License
 *
 * Copyright (c) 2020 Thorsten Klose (tk@midibox.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * =============================================================================
 */

#ifndef _MIDI_H
#define _MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  MIDI_IF_UART = 0,
  MIDI_IF_BLE,
  MIDI_IF_APPLE,
} midi_if_t;

extern uint8_t midi_if_encode_port(uint8_t midi_if, uint8_t midi_port);
extern midi_if_t midi_if_get_type(uint8_t midi_port);
extern uint8_t midi_if_get_port(uint8_t midi_port);

extern int32_t midi_init(void *callback_midi_message_received);

extern int32_t midi_send_event1(uint8_t midi_port, uint8_t evnt0);
extern int32_t midi_send_event2(uint8_t midi_port, uint8_t evnt0, uint8_t evnt1);
extern int32_t midi_send_event3(uint8_t midi_port, uint8_t evnt0, uint8_t evnt1, uint8_t evnt2);

extern int32_t midi_send_sysex_dump(uint8_t midi_port, uint8_t sysex_device_id, uint8_t *response, size_t response_len, int checksum_start);

#ifdef __cplusplus
}
#endif

#endif /* _MIDI_H */
