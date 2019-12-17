/* \defgroup MFDRV
 *
 * Motorfader Driver
 *
 * See README.md for usage hints
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

#ifndef _MFDRV_H
#define _MFDRV_H

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
// Global definitions
/////////////////////////////////////////////////////////////////////////////

// number of motorfaders (0-8)
#ifndef MFDRV_NUM
#define MFDRV_NUM 8
#endif


/////////////////////////////////////////////////////////////////////////////
// Global Types
/////////////////////////////////////////////////////////////////////////////

typedef enum {
  MF_Standby = 0,
  MF_Up      = 1,
  MF_Down    = 2
} mfdrv_direction_t;

typedef union {
  struct {
    uint8_t ALL;
  };
  struct {
    uint8_t enabled:1;
    uint8_t inverted:1;
  };
} mfdrv_fader_mode_t;

typedef struct {
  uint16_t fader_min_value;
  uint16_t fader_max_value;
  mfdrv_fader_mode_t fader_mode;
  uint8_t  ain_deadband;
  uint8_t  mf_deadband;
  uint8_t  pwm_period;
  uint8_t  pwm_duty_cycle_up_min;
  uint8_t  pwm_duty_cycle_up_max;
  uint8_t  pwm_duty_cycle_down_min;
  uint8_t  pwm_duty_cycle_down_max;
} mfdrv_config_t;


/////////////////////////////////////////////////////////////////////////////
// Prototypes
/////////////////////////////////////////////////////////////////////////////

extern int32_t MFDRV_Init(void);

extern int32_t MFDRV_FaderMove(uint32_t mf, uint16_t pos);
extern int32_t MFDRV_FaderDirectMove(uint32_t mf, mfdrv_direction_t direction);
extern int32_t MFDRV_FaderPositionGet(uint32_t mf);

extern int32_t MFDRV_TraceRequest(uint32_t mf, uint8_t scale);
extern int32_t MFDRV_TraceAvailable(uint16_t **trace);

extern int32_t MFDRV_SuspendSet(uint32_t mf, uint8_t suspend);
extern int32_t MFDRV_SuspendGet(uint32_t mf);

extern int32_t MFDRV_TouchDetectionReset(uint32_t mf);
extern int32_t MFDRV_TouchSensorGet(uint32_t ts);

extern int32_t MFDRV_ConfigSet(uint32_t mf, mfdrv_config_t config);
extern mfdrv_config_t MFDRV_ConfigGet(uint32_t mf);

extern int32_t MFDRV_Tick(uint16_t *ain_values, uint8_t *dinsr_values, void *_callback_manual_fader_change, void *_callback_touch_sensor_change);


/////////////////////////////////////////////////////////////////////////////
// Export global variables
/////////////////////////////////////////////////////////////////////////////


#endif /* _MFDRV_H */
