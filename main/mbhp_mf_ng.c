/*
 * MBHP_MF_NG for ESP32
 *
 * See ../README.md for usage hints
 *
 * =============================================================================
 *
 * MIT License
 *
 * Copyright (c) 2019 Thorsten Klose (tk@midibox.org)
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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/gpio_struct.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "console.h"
#include "midi.h"
#include "uartmidi.h"

#include "mfdrv.h"


#define TAG "MIDIbox"


// IO Pins for SPI connections
#define PIN_NUM_MISO_ADC   35 // instead of standard D19; use this input-only pin for MISO to get D19 for PWM
#define PIN_NUM_MISO_DINSR 34 // input for 74HC165
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

uint8_t midi_port_for_traces = 0x00;
uint8_t sysex_device_id = 0;

typedef union {
  struct {
    uint8_t ALL[256];
  };
  struct {
    uint8_t name[16];
    uint8_t number_of_faders; // 0x10
    uint8_t operation_mode; // 0x11
    uint8_t merger_mode; // 0x12
    uint8_t pwm_steps; // 0x13
    uint8_t ain_deadband; // 0x14
    uint8_t mf_deadband; // 0x15
    uint8_t touch_sensor_mode; // 0x16
    uint8_t touch_sensor_sensitivity; // 0x17
    uint8_t midi_channel; // 0x18
    uint8_t reserved[7]; // 0x19..0x1f: makes 32 bytes up to here

    uint8_t fader_mode[8];  // 0x20..0x27
    uint8_t min_value_l[8]; // 0x28..0x2f - only low byte is stored
    uint8_t max_value_l[8]; // 0x30..0x37 - only low byte is stored
    uint8_t min_duty_up[8]; // 0x38..0x3f
    uint8_t max_duty_up[8]; // 0x40..0x47
    uint8_t min_duty_down[8]; // 0x48..0x4f
    uint8_t max_duty_down[8]; // 0x50..0x57
    uint8_t reserved2[8];     // 0x58..0x5f
    
    uint8_t reserved_0x6x[16];
    uint8_t reserved_0x7x[16];
    uint8_t reserved_0x8x[16];
    uint8_t reserved_0x9x[16];
    uint8_t reserved_0xax[16];
    uint8_t reserved_0xbx[16];
    uint8_t reserved_0xcx[16];
    uint8_t reserved_0xdx[16];
    uint8_t reserved_0xex[16];
    uint8_t reserved_0xfx[16];
  };
} patch_t;

patch_t patch = { // currently only a single patch is supported
  .name = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  .number_of_faders = MFDRV_NUM,
  .operation_mode = 0,
  .merger_mode = 0,
  .pwm_steps = 64,
  .ain_deadband = 3,
  .mf_deadband = 3,
  .touch_sensor_mode = 1,
  .touch_sensor_sensitivity = 3,
  .midi_channel = 0,
  .fader_mode = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
  .min_value_l = {20, 20, 20, 20, 20, 20, 20, 20},
  .max_value_l = {255, 255, 255, 255, 255, 255, 255, 255},
  .min_duty_up = {10, 10, 10, 10, 10, 10, 10, 10},
  .max_duty_up = {64, 64, 64, 64, 64, 64, 64, 64},
  .min_duty_down = {10, 10, 10, 10, 10, 10, 10, 10},
  .max_duty_down = {64, 64, 64, 64, 64, 64, 64, 64},
};


////////////////////////////////////////////////////////////////////////////////////////////////////
// Writes into patch and transfers to application
////////////////////////////////////////////////////////////////////////////////////////////////////
static int32_t sysex_patch_write(uint8_t address, uint8_t value)
{
  patch.ALL[address] = value;

  // transfer to MF driver
  if( address < 0x20 ) { // below MF configuration area
    switch( address ) { // only parameters which have to be forwarded to MF Driver are handled here
    case 0x13: { // PWM Steps
      // TODO
    } break;
      
    case 0x14: { // AIN Deadband
      int mf;
      for(mf=0; mf<MFDRV_NUM; ++mf) {
        mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
        cfg.ain_deadband = value << 2; // 12bit -> 14bit to keep compatibility with MBHP_MF_NG
        MFDRV_ConfigSet(mf, cfg);
      }
    } break;
      
    case 0x15: { // MF Deadband
      int mf;
      for(mf=0; mf<MFDRV_NUM; ++mf) {
        mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
        cfg.mf_deadband = value << 2; // 12bit -> 14bit to keep compatibility with MBHP_MF_NG
        MFDRV_ConfigSet(mf, cfg);
      }
    } break;
    }
  } else { // MF configuration area
    uint32_t mf = address & 0x7;
    switch( address & 0xf8 ) {
      
    case 0x20: { // Fader Mode
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.fader_mode.ALL = value;
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    case 0x28: { // Min Value
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.fader_min_value = (uint16_t)value << 2; // only 8bit... and we convert from 10bit to 12bit to keep compatibility with MBHP_MF_NG
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    case 0x30: { // Max Value
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.fader_max_value = (0x1000 - 4*256) + ((uint16_t)value << 2); // only 8bit... and we convert from 10bit to 12bit to keep compatibility with MBHP_MF_NG
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    case 0x38: { // Min Duty Up
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.pwm_duty_cycle_up_min = value;
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    case 0x40: { // Max Duty Up
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.pwm_duty_cycle_up_max = value;
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    case 0x48: { // Min Duty Down
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.pwm_duty_cycle_down_min = value;
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    case 0x50: { // Max Duty Down
      mfdrv_config_t cfg = MFDRV_ConfigGet(mf);
      cfg.pwm_duty_cycle_down_max = value;
      MFDRV_ConfigSet(mf, cfg);
    } break;
      
    }
  }
  
  return 0; // no error
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// This callback is called whenever a new MIDI message is received
////////////////////////////////////////////////////////////////////////////////////////////////////
static void callback_midi_message_received(uint8_t midi_port, uint16_t timestamp, uint8_t midi_status, uint8_t *remaining_message, size_t len, size_t continued_sysex_pos)
{
  ESP_LOGI(TAG, "CALLBACK midi_port=0x%02x, timestamp=%d, midi_status=0x%02x, len=%d, continued_sysex_pos=%d, remaining_message:", midi_port, timestamp, midi_status, len, continued_sysex_pos);
  esp_log_buffer_hex(TAG, remaining_message, len);

  // SysEx communication
  // Note: parser has to consider that incoming SysEx stream could be received piece-wise
  static uint8_t sysex_buffer[600]; // this big SysEx buffer enormously simplifies parsing of incoming SysEx data chunks!
  static size_t sysex_buffer_len = 0;
  static uint8_t sysex_buffer_port = 0xff; // TODO: currently we interrupt if SysEx is received from another port in parallel - could be improved; e.g. header matching: only listen to that port until f7

  if( midi_status == 0xf0 ) {
    if( sysex_buffer_port != 0xff && midi_port != sysex_buffer_port )
      return; // ignore - TODO: timeout!
    sysex_buffer_port = midi_port; // got it

    size_t copy_len = len;
    if( (continued_sysex_pos + copy_len) > sizeof(sysex_buffer) ) {
      copy_len = (continued_sysex_pos > sizeof(sysex_buffer)) ? 0 : sizeof(sysex_buffer) - continued_sysex_pos;
    }
    memcpy(&sysex_buffer[continued_sysex_pos], remaining_message, copy_len);

    if( continued_sysex_pos == 0 ) {
      sysex_buffer_len = copy_len;
    } else {
      sysex_buffer_len += copy_len;
    }
  } else if( midi_status == 0xf7 ) {
    if( midi_port != sysex_buffer_port )
      return; // ignore - TODO: timeout!
    sysex_buffer_port = 0xff; // message has been handled, allow other MIDI ports to fill the buffer

    uint8_t header[5] = { 0x00, 0x00, 0x7e, 0x4f, sysex_device_id };
    if( sysex_buffer_len < sizeof(header) || memcmp(header, sysex_buffer, sizeof(header)) != 0 ) {
      return; // header not matching
    }

    uint8_t *sysex_stream = &sysex_buffer[sizeof(header)];
    switch( *(sysex_stream++) ) {
    case 0x1: { // Patch Read
      if( sysex_buffer_len < (sizeof(header) + 1 + 1) ) {
        uint8_t response[2] = {0xe, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Error Response
        return;
      }

      int i;
      uint8_t patch_number = *(sysex_stream++); // TODO: handle patch number

      const size_t response_len = 1+1+512;
      uint8_t response[response_len];
      uint8_t *response_ptr = response;
	
      *(response_ptr++) = 0x02; // Patch Write
      *(response_ptr++) = patch_number;

      for(i=0; i<sizeof(patch_t); ++i) {
        *(response_ptr++) = patch.ALL[i] & 0xf;
        *(response_ptr++) = patch.ALL[i] >> 4;
      }

      midi_send_sysex_dump(midi_port, sysex_device_id, response, response_len, 2);
    } break;
	
    case 0x2: { // Patch Write
      if( sysex_buffer_len < (sizeof(header) + 1 + 512 + 1) ) {
        uint8_t response[2] = {0xe, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Error Response
        return;
      }

      static uint8_t patch_number = 0;
      static uint8_t checksum = 0x00;

      int start_pos;
      if( !continued_sysex_pos ) {
        patch_number = *(sysex_stream++); // TODO: handle patch number
        checksum = 0;
        start_pos = 0;
      } else {
        start_pos = continued_sysex_pos - 7;
      }
      
      uint8_t *checksum_scan_ptr = sysex_stream;
      int i;
      for(i=start_pos; i<2*sizeof(patch_t); ++i) {
        // transfer value to patch structure
        if( (i % 2) == 0 ) {
          patch.ALL[i/2] &= 0xf0;
          patch.ALL[i/2] |= *checksum_scan_ptr & 0x0f;
        } else {
          patch.ALL[i/2] &= 0x0f;
          patch.ALL[i/2] |= (*checksum_scan_ptr & 0x0f) << 4;
        }
	
        checksum += *(checksum_scan_ptr++);
      }
      uint8_t expected_checksum = (0x80 - checksum) & 0x7f;

      if( expected_checksum == *checksum_scan_ptr ) {
        // if checksum matching: transfer patch structure to MFDRV
        int i;
        for(i=0; i<sizeof(patch_t); ++i) {
          sysex_patch_write(i, patch.ALL[i]);
        }
      }
      
      uint8_t response[2] = {(expected_checksum == *checksum_scan_ptr) ? 0xf : 0xe, expected_checksum};
      midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Ack or Error Response
    } break;
	
    case 0x6: { // Parameter Write
      if( sysex_buffer_len < (sizeof(header) + 1 + 2 + 2) ) {
        uint8_t response[2] = {0xe, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Error Response
        return;
      }

      uint32_t address = (*(sysex_stream++) & 0x01) << 7;
      address |= (*(sysex_stream++) & 0x7f);

      int num_bytes = (sysex_buffer_len-7) / 2;
      int i;
      for(i=0; i<num_bytes; ++i, ++address) {
        uint8_t data = (*(sysex_stream++) & 0x0f);
        data |= (*(sysex_stream++) & 0x0f) << 4;
        //printf("%d: 0x%02x = 0x%02x\n", i, address, data);

        if( address < sizeof(patch_t) ) {
          sysex_patch_write(address, data);
        }
      }

      {
        uint8_t response[2] = {0xf, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Ack
      }
    } break;
	
    case 0x9: { // Faders Get
      if( sysex_buffer_len < (sizeof(header) + 1) ) {
        uint8_t response[2] = {0xe, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Error Response
        return;
      }

      uint8_t first_fader = *(sysex_stream++);

      {
        uint8_t response[2+16];
        uint8_t *response_ptr = response;
        *(response_ptr++) = 0x0a; // like "Faders Set"
        *(response_ptr++) = first_fader;

        int i;
        for(i=first_fader; i<8; ++i) {
          uint16_t pos = MFDRV_FaderPositionGet(i);
          *(response_ptr++) = pos & 0x7f;
          *(response_ptr++) = pos >> 7;
        }
	  
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2+2*(8-first_fader), -1); // Ack
      }
    } break;
	
    case 0xa: { // Faders Set
      if( sysex_buffer_len < (sizeof(header) + 1 + 2) ) {
        uint8_t response[2] = {0xe, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Error Response
        return;
      }
      uint8_t first_fader = *(sysex_stream++);

      int i;
      int num_faders = (sysex_buffer_len-7) / 2;
      for(i=0; i<num_faders; ++i) {
        uint16_t pos = *(sysex_stream++);
        pos |= ((uint16_t)*(sysex_stream++)) << 7;

        uint8_t mf = i + first_fader;
        MFDRV_TouchDetectionReset(mf);
        MFDRV_FaderMove(mf, pos << 2); // convert to MBHP_MF_NG resolution
      }

      {
        uint8_t response[2] = {0xf, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Ack
      }
    } break;
	
    case 0xb: { // Trace Request
      if( sysex_buffer_len < (sizeof(header) + 1) ) {
        uint8_t response[2] = {0xe, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Error Response
        return;
      }

      uint8_t mf = *(sysex_stream++);
      uint8_t scale = *(sysex_stream++);
      midi_port_for_traces = midi_port;
      MFDRV_TraceRequest(mf, scale);
    } break;
	
    case 0xf: { // Ping
      // feedback detection: only send back if no additional byte has been received!
      if( sysex_buffer_len == 6 ) {
        uint8_t response[2] = {0xf, 0x00};
        midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // Pong
      }
    } break;
    default: { // Invalid Command
      uint8_t response[2] = {0xe, 0x00};
      midi_send_sysex_dump(midi_port, sysex_device_id, response, 2, -1); // invalid command
    }
    }
  }
    

  // now handle different fader modes
  // TOOD: ... currently only pitch bend supported!
  if( (midi_status & 0xf0) == 0xe0 ) {
    uint32_t mf = midi_status & 0xf;
    uint16_t value = remaining_message[0] | ((uint16_t)remaining_message[1] << 7);

    if( mf < MFDRV_NUM ) {
      MFDRV_FaderMove(mf, (uint16_t)value >> 2); // 14bit -> 12bit
    }
  }
}




////////////////////////////////////////////////////////////////////////////////////////////////////
// Called whenever a motorfader is moved manually
////////////////////////////////////////////////////////////////////////////////////////////////////
static void callback_manual_fader_change(uint32_t mf, uint16_t value)
{
  // send PitchBend event
  // TODO: consider patch.operation_mode
  // TODO: more comfortable packet creation via special APIs

  uint8_t send = 1;
  if( patch.touch_sensor_mode >= 3 && MFDRV_TouchSensorGet(mf) != 0 ) { // only send if touch sensor active
    send = 0; // sensor not touched (value != 0) -> don't send
  }

  if( send ) {
    // 12bit -> 14bit value
    value <<= 2;

    midi_send_event3(midi_if_encode_port(MIDI_IF_UART, 2), 0xe0 + mf, value & 0x7f, value >> 7);
    midi_send_event3(midi_if_encode_port(MIDI_IF_BLE, 0), 0xe0 + mf, value & 0x7f, value >> 7);
    midi_send_event3(midi_if_encode_port(MIDI_IF_APPLE, 1), 0xe0 + mf, value & 0x7f, value >> 7);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Called whenever a touch sensor is activated.
// Note: the value is typically low-active, means: if 0, sensor is touched)
////////////////////////////////////////////////////////////////////////////////////////////////////
static void callback_touch_sensor_change(uint32_t ts, uint8_t value)
{
  //printf("Touch Sensor #%d %s\n", ts+1, value ? "untouched" : "touched");

  if( patch.touch_sensor_mode >= 1 ) { // touch sensor will generate a MIDI event
    // TODO: make MIDI event optional depending on patch.operation_mode    
    midi_send_event3(midi_if_encode_port(MIDI_IF_UART, 2), 0x90, 0x68 + ts, value ? 0x00 : 0x7f);
    midi_send_event3(midi_if_encode_port(MIDI_IF_BLE, 0), 0x90, 0x68 + ts, value ? 0x00 : 0x7f);
    midi_send_event3(midi_if_encode_port(MIDI_IF_APPLE, 1), 0x90, 0x68 + ts, value ? 0x00 : 0x7f);
  }

  if( patch.touch_sensor_mode >= 2 ) { // touch sensor will suspend motor
    MFDRV_SuspendSet(ts, value ? 0 : 1);
  }
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// This task is periodically called to convert fader values + touch sensors, and control the motors
////////////////////////////////////////////////////////////////////////////////////////////////////
static void task_mf(void *pvParameters)
{
  spi_device_handle_t spi_adc;
  esp_err_t ret;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // SPI Initialization
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // we need two separate busses for the same SPI
  // because ADC and DIN SR are connected to different MISO pins
  spi_bus_config_t buscfg_adc = {
        .miso_io_num=PIN_NUM_MISO_ADC,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=8
  };

  spi_bus_config_t buscfg_dinsr = {
        .miso_io_num=PIN_NUM_MISO_DINSR,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=1
  };

  // the device config
  // note: originally I wanted to define a separate one for dinsr, but unfortunately the driver doesn't allow to switch the CS polarity during runtime
  // as a workaround we switch the polarity in the HW registers directory whenever required
  spi_device_interface_config_t devcfg_adc = {
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=1*1000*1000,           //Clock out at 1 MHz
        .duty_cycle_pos=128,
        .mode=0,
        .spics_io_num=PIN_NUM_CS,
        .queue_size=8
    };

#if 0  
  spi_device_interface_config_t devcfg_dinsr = {
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=1*1000*1000,           //Clock out at 1 MHz
        .duty_cycle_pos=128,
        .mode=0,
	.flags = SPI_DEVICE_POSITIVE_CS,
        .spics_io_num=PIN_NUM_CS,
        .queue_size=1
    };
#endif
  
  // Initialize the SPI bus for ADC
  ret=spi_bus_initialize(VSPI_HOST, &buscfg_adc, 0);
  ESP_ERROR_CHECK(ret);

  // Attach the ADC to the SPI bus
  ret=spi_bus_add_device(VSPI_HOST, &devcfg_adc, &spi_adc);
  ESP_ERROR_CHECK(ret);

  // prepare ADC transactions 
  uint8_t adc_send_buffer[8][3];
  uint8_t adc_receive_buffer[8][3];
  spi_transaction_t spi_transaction_adc[8];
  {
    memset(&spi_transaction_adc, 0, sizeof(spi_transaction_adc));

    int i;
    for(i=0; i<8; ++i) {
#if 0      
      uint8_t chn = i;
#else
      uint8_t chn = 7-i; // to simplify layout the channel order has been mirrored
#endif      
      // retrieve conversion values
      // shift in start bit + SGL + MSB of channel selection, shift out dummy byte
      adc_send_buffer[i][0] = 0x6 | (chn >> 2);
      // shift in remaining 2 bits of channel selection, shift out MSBs of conversion value
      adc_send_buffer[i][1] = chn << 6;
      // shift in dummy, shift out LSBs of conversion value
      adc_send_buffer[i][2] = 0x00;

      spi_transaction_adc[i].length = 3*8;
      spi_transaction_adc[i].tx_buffer = adc_send_buffer[i];
      spi_transaction_adc[i].rx_buffer = adc_receive_buffer[i];
    }
  }


  // prepare DINSR transactions 
  uint8_t dinsr_receive_buffer[1];
  spi_transaction_t spi_transaction_dinsr[1];
  {
    memset(&spi_transaction_dinsr, 0, sizeof(spi_transaction_dinsr));

    spi_transaction_dinsr[0].length = 1*8;
    spi_transaction_dinsr[0].tx_buffer = NULL;
    spi_transaction_dinsr[0].rx_buffer = &dinsr_receive_buffer[0];
  }


  
  //////////////////////////////////////////////////////////////////////////////////////////////////
  // initialize Motorfader Driver
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // the driver itself
  MFDRV_Init();

  // then restore configuration values from patch structure
  {
    int i;
    for(i=0; i<sizeof(patch_t); ++i) {
      sysex_patch_write(i, patch.ALL[i]);
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Finally: the endless running control loop
  //////////////////////////////////////////////////////////////////////////////////////////////////
  while( 1 ) {
    int i;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // increase MIDI Timestamps
    ////////////////////////////////////////////////////////////////////////////////////////////////
    midi_tick();


    ////////////////////////////////////////////////////////////////////////////////////////////////
    // ADC Conversions
    ////////////////////////////////////////////////////////////////////////////////////////////////
    for(i=0; i<8; ++i) {
      ret = spi_device_queue_trans(spi_adc, &spi_transaction_adc[i], portMAX_DELAY);
      assert(ret==ESP_OK);
    }

    // keep SPI transfer running, get back in 1 mS when results should be available
    vTaskDelay( 1 * portTICK_PERIOD_MS );

    // get results from receive buffers
    uint16_t adc_results[8];
    for(i=0; i<8; ++i) {
      spi_transaction_t *ret_trans;
      ret = spi_device_get_trans_result(spi_adc, &ret_trans, portMAX_DELAY);
      assert(ret==ESP_OK);
      assert(ret_trans == &spi_transaction_adc[i]);
	
      adc_results[i] = ((adc_receive_buffer[i][1] & 0xf) << 8) | (adc_receive_buffer[i][2]);
    }

#if 0
    // for debugging...
    printf("0x%03x 0x%03x 0x%03x 0x%03x 0x%03x 0x%03x 0x%03x 0x%03x\n",
	   adc_results[0], adc_results[1], adc_results[2], adc_results[3],
	   adc_results[4], adc_results[5], adc_results[6], adc_results[7]);
#endif      
    
    
    // DINSR
    {
      static unsigned ctr = 0;

      if( ++ctr == 100 ) {
        ctr = 0;

        uint32_t dummy;
        spicommon_bus_free_io_cfg(&buscfg_adc);
        spicommon_bus_initialize_io(VSPI_HOST, &buscfg_dinsr, 0, SPICOMMON_BUSFLAG_MASTER, &dummy);
        spicommon_hw_for_host(VSPI_HOST)->pin.master_cs_pol |= (1 << 0);
 
        ret = spi_device_transmit(spi_adc, &spi_transaction_dinsr[0]);
        assert(ret==ESP_OK);

        spicommon_bus_free_io_cfg(&buscfg_dinsr);
        spicommon_bus_initialize_io(VSPI_HOST, &buscfg_adc, 0, SPICOMMON_BUSFLAG_MASTER, &dummy);
        spicommon_hw_for_host(VSPI_HOST)->pin.master_cs_pol &= ~(1 << 0);
      }
    }

    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // pass new ADC and DIN values to Motorfader Driver
    ////////////////////////////////////////////////////////////////////////////////////////////////    

    // this one is doing the magic...
    MFDRV_Tick(adc_results, dinsr_receive_buffer, callback_manual_fader_change, callback_touch_sensor_change);

    // in case trace is enabled: send trace data via SysEx to MIOS Studio
    {
      uint16_t *trace = NULL;
      if( MFDRV_TraceAvailable(&trace) ) {
        const size_t response_len = 1+512;
        uint8_t response[response_len];
        uint8_t *response_ptr = response;
	
        *(response_ptr++) = 0x0c; // Trace dump

        for(i=0; i<sizeof(patch_t); ++i) {
          uint16_t pos = trace[i] >> 2; // convert back to MBHP_MF_NG resolution
          *(response_ptr++) = pos & 0x7f;
          *(response_ptr++) = pos >> 7;
        }
	
        midi_send_sysex_dump(midi_port_for_traces, sysex_device_id, response, response_len, -1);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Handling the Console in an independent task
////////////////////////////////////////////////////////////////////////////////////////////////////
static void console_task(void *pvParameters)
{
  console_init();

  while( 1 ) {
    console_tick();
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// Application Starting Point
////////////////////////////////////////////////////////////////////////////////////////////////////
void app_main()
{
  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  // log levels
  esp_log_level_set("gpio", ESP_LOG_WARN);
  esp_log_level_set("MIDIbox", ESP_LOG_WARN); // can be changed with midibox_debug in console

  // start with random seed
  srand(esp_random());

  // initialize MIDI interfaces
  midi_init(callback_midi_message_received);

  // launch tasks
  if( uartmidi_get_enabled(2) == 0 ) {
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
  }

  xTaskCreate(task_mf, "task_mf", 4096, NULL, 8, NULL);
}
