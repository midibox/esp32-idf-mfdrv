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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "midi.h"
#include "wifi.h"
#include "blemidi.h"
#include "applemidi.h"
#include "if/lwip/applemidi_if.h"

#define TAG "MIDIbox"

static void (*callback_midi_message_received)(uint8_t midi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos);

////////////////////////////////////////////////////////////////////////////////////////////////////
// MIDI Port encoding/decoding
////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t midi_if_encode_port(uint8_t midi_if, uint8_t midi_port)
{
  return (uint8_t)(midi_if << 4) | (midi_port & 0x0f);
}

uint8_t midi_if_get_port(uint8_t midi_port)
{
  return (midi_port & 0x0f);
}

midi_if_t midi_if_get_type(uint8_t midi_port)
{
  return (midi_if_t)(midi_port >> 4); // encoded in higher nibble
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// This callback is called whenever a new BLE MIDI message is received
////////////////////////////////////////////////////////////////////////////////////////////////////
static void blemidi_callback_midi_message_received(uint8_t blemidi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  ESP_LOGI(TAG, "CALLBACK blemidi_port=%d, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", blemidi_port, timestamp, midi_status, len, continued_sysex_pos);
  esp_log_buffer_hex(TAG, remaining_message, len);

  if( callback_midi_message_received ) {
    callback_midi_message_received(midi_if_encode_port(MIDI_IF_BLE, blemidi_port), timestamp, midi_status, remaining_message, len, continued_sysex_pos);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// This function is called from the Apple MIDI Driver whenever a new MIDI message has been received
////////////////////////////////////////////////////////////////////////////////////////////////////
static void applemidi_callback_midi_message_received(uint8_t applemidi_port, uint32_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  if( applemidi_get_debug_level() >= 3 ) {
    // Note: with these messages enabled, we potentially get packet loss!
    ESP_LOGI(TAG, "receive_packet CALLBACK applemidi_port=%d, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", applemidi_port, timestamp, midi_status, len, continued_sysex_pos);
    esp_log_buffer_hex(TAG, remaining_message, len);
  }

  if( callback_midi_message_received ) {
    callback_midi_message_received(midi_if_encode_port(MIDI_IF_APPLE, applemidi_port), timestamp, midi_status, remaining_message, len, continued_sysex_pos);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// WIFI Connection + Apple MIDI Handling
////////////////////////////////////////////////////////////////////////////////////////////////////
static void udp_task(void *pvParameters)
{
  wifi_init();

  while( 1 ) {
    if( !wifi_connected() ) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    } else {
      applemidi_if_init(APPLEMIDI_DEFAULT_PORT);
      applemidi_init(applemidi_callback_midi_message_received, applemidi_if_send_udp_datagram);

      while( wifi_connected() ) {
        applemidi_if_tick(applemidi_parse_udp_datagram);
        applemidi_tick();
      }

      applemidi_if_deinit();
    }
  }

  vTaskDelete(NULL);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize MIDI interfaces
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t midi_init(void *_callback_midi_message_received)
{
  callback_midi_message_received = _callback_midi_message_received;

  xTaskCreate(udp_task, "udp", 4096, NULL, 5, NULL);

  blemidi_init(blemidi_callback_midi_message_received);

  return 0; // no error
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Sends a common MIDI message
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t midi_send_event1(uint8_t midi_port, uint8_t evnt0)
{
  uint8_t message[1] = {evnt0};

  switch( midi_if_get_type(midi_port) ) {

  case MIDI_IF_BLE: {
    blemidi_send_message(midi_if_get_port(midi_port), message, sizeof(message));
  } break;

  case MIDI_IF_APPLE: {
    applemidi_send_message(midi_if_get_port(midi_port), message, sizeof(message));
  } break;

  default: {
  }
  }

  return 0;
}

int32_t midi_send_event2(uint8_t midi_port, uint8_t evnt0, uint8_t evnt1)
{
  uint8_t message[2] = {evnt0, evnt1};

  switch( midi_if_get_type(midi_port) ) {
  case MIDI_IF_BLE: {
    blemidi_send_message(midi_if_get_port(midi_port), message, sizeof(message));
  } break;

  case MIDI_IF_APPLE: {
    applemidi_send_message(midi_if_get_port(midi_port), message, sizeof(message));
  } break;

  default: {
  }
  }

  return 0;
}

int32_t midi_send_event3(uint8_t midi_port, uint8_t evnt0, uint8_t evnt1, uint8_t evnt2)
{
  uint8_t message[3] = {evnt0, evnt1, evnt2 };

  switch( midi_if_get_type(midi_port) ) {
  case MIDI_IF_BLE: {
    blemidi_send_message(midi_if_get_port(midi_port), message, sizeof(message));
  } break;

  case MIDI_IF_APPLE: {
    applemidi_send_message(midi_if_get_port(midi_port), message, sizeof(message));
  } break;

  default: {
  }
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Sends a SysEx dump with device specific header
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t midi_send_sysex_dump(uint8_t midi_port, uint8_t sysex_device_id, uint8_t *response, size_t response_len, int checksum_start)
{
  uint8_t is_applemidi = midi_if_get_type(midi_port) == MIDI_IF_APPLE;
  size_t message_len = 6 + response_len; // includes MIDI status and remaining bytes

  if( checksum_start >= 0 )
    message_len += 1; // + checksum

  uint8_t *message = (uint8_t *)malloc(message_len * sizeof(uint8_t) + 1); // +1 for potential applemidi packet (see MIDI_IF_APPLE)
  size_t pos = 0;
  message[pos++] = 0xf0;
  message[pos++] = 0x00;
  message[pos++] = 0x00;
  message[pos++] = 0x7e;
  message[pos++] = 0x4f; // MBHP_MF_NG
  message[pos++] = sysex_device_id;
  memcpy(&message[pos], response, response_len);

  if( checksum_start >= 0 ) {
    int i;
    uint8_t checksum = 0x00;

    for(i=checksum_start; i<response_len-1; ++i) {
      checksum += response[i];
    }
    message[message_len-1] = (0x80 - checksum) & 0x7f;
  }

  switch( midi_if_get_type(midi_port) ) {
  case MIDI_IF_BLE: {
    blemidi_send_message(midi_if_get_port(midi_port), message, message_len);

    uint8_t f7 = 0xf7;
    blemidi_send_message(midi_if_get_port(midi_port), &f7, 1);
  } break;

  case MIDI_IF_APPLE: {
    if( is_applemidi ) {
      message_len += 1; // 0xf7 has to be part of the message, otherwise we can't comply to Apple MIDI protocol
      message[message_len-1] = 0xf7;
    }

    applemidi_send_message(midi_if_get_port(midi_port), message, message_len);
  } break;

  default: {
  }
  }

  free(message);

  return 0; // no error
}
