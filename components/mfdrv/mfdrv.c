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

/////////////////////////////////////////////////////////////////////////////
// Include files
/////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>

#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


#include "mfdrv.h"

/////////////////////////////////////////////////////////////////////////////
// Local defines/macros
/////////////////////////////////////////////////////////////////////////////

#define REPEAT_CTR_RELOAD         31  // retries to reach target position
#define TIMEOUT_CTR_RELOAD       255 // give up after how many mS
#define MANUAL_MOVE_CTR_RELOAD   255 // ignore new position request for how many mS
#define SLOWDOWN_THRESHOLD       512


#define MFDRV_PWM_CH_NUM         (2*MFDRV_NUM)


const ledc_channel_config_t ledc_channel[MFDRV_PWM_CH_NUM] = {
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_0, .gpio_num = 13, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF1 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_0, .gpio_num = 12, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF1 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_1, .gpio_num = 14, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF2 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_1, .gpio_num = 27, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF2 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_2, .gpio_num = 26, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF3 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_2, .gpio_num = 25, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF3 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_3, .gpio_num = 33, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF4 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_3, .gpio_num = 32, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF4 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_4, .gpio_num = 15, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF5 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_4, .gpio_num =  2, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF5 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_5, .gpio_num =  4, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF6 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_5, .gpio_num = 16, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF6 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_6, .gpio_num = 17, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF7 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_6, .gpio_num = 19, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF7 Down
  { .timer_sel = LEDC_TIMER_0, .channel = LEDC_CHANNEL_7, .gpio_num = 21, .speed_mode = LEDC_HIGH_SPEED_MODE }, // MF8 Up
  { .timer_sel = LEDC_TIMER_1, .channel = LEDC_CHANNEL_7, .gpio_num = 22, .speed_mode = LEDC_LOW_SPEED_MODE  }, // MF8 Down
};


/////////////////////////////////////////////////////////////////////////////
// Local types
/////////////////////////////////////////////////////////////////////////////

typedef union {
  struct {
    uint32_t ALL0:32;
    uint32_t ALL1:32;
    uint32_t ALL2:32;
  };
  struct {
#if 0  // bitfields don't work with ESP32 - values will be overwritten ?!?
    uint32_t up:1;
    uint32_t down:1;
    uint32_t suspended:1;
    uint32_t idle:1;
    uint32_t direct_control:1;
    uint32_t dummy:3+8; // fill to 16bit
#else
    uint8_t up;
    uint8_t down;
    uint8_t suspended;
    uint8_t idle;
    uint8_t direct_control;
#endif
    
    uint16_t target_pos;
    uint16_t current_pos;
    uint8_t  current_ts_value;

    uint8_t  duty_cycle;
    uint8_t  manual_move_ctr;
    uint8_t  timeout_ctr;
    uint8_t  repeat_ctr;

    mfdrv_config_t config;
  };
  struct {
    mfdrv_direction_t direction;
  };
} mf_state_t;

typedef enum {
  MF_TRACE_STATE_IDLE = 0,
  MF_TRACE_STATE_ARMED,
  MF_TRACE_STATE_RUNNING,
  MF_TRACE_STATE_FINISHED,
  MF_TRACE_STATE_SENT
} mf_trace_state_t;

typedef struct {
  mf_trace_state_t state;
  uint8_t fader;
  uint8_t scale;
  uint8_t scale_ctr;
  uint16_t pos;
  uint16_t data[256];
} mf_trace_t;


/////////////////////////////////////////////////////////////////////////////
// Local variables
/////////////////////////////////////////////////////////////////////////////

#if MFDRV_NUM

// control variables for motorfaders
static mf_state_t mf_state[MFDRV_NUM];
static mf_trace_t mf_trace;
#endif


/////////////////////////////////////////////////////////////////////////////
// Prototypes
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//! Initializes motorfader driver
//! \return < 0 if initialisation failed
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_Init(void)
{
#if !MFDRV_NUM
  return -1; // no motorfaders
#else

  int i;
  for(i=0; i<MFDRV_NUM; ++i) {
    mf_state[i].ALL0 = 0;
    mf_state[i].ALL1 = 0;
    mf_state[i].ALL2 = 0;

    mf_state[i].config.fader_mode.ALL = 0;
    mf_state[i].config.fader_mode.enabled = 1;
    mf_state[i].config.ain_deadband = 15;
    mf_state[i].config.mf_deadband = 15;
    mf_state[i].config.fader_min_value = 80;
    mf_state[i].config.fader_max_value = 4000;
    mf_state[i].config.pwm_period = 64;
    mf_state[i].config.pwm_duty_cycle_down_max = 64;
    mf_state[i].config.pwm_duty_cycle_down_min = 32;
    mf_state[i].config.pwm_duty_cycle_up_max = 64;
    mf_state[i].config.pwm_duty_cycle_up_min = 32;
  }

  mf_trace.state = MF_TRACE_STATE_IDLE;

  // PWM via LEDC driver
  ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_6_BIT,   // resolution of PWM duty
    .freq_hz = 200000,                     // frequency of PWM signal (currently hardcoded)
    .speed_mode = LEDC_HIGH_SPEED_MODE,    // timer mode
    .timer_num = LEDC_TIMER_0              // timer index
  };
  
  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);

  // Prepare and set configuration of timer1 for low speed channels
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.timer_num = LEDC_TIMER_1;
  ledc_timer_config(&ledc_timer);

  // Set LED Controller with previously prepared configuration
  {
    int ch;
    for (ch = 0; ch < MFDRV_PWM_CH_NUM; ch++) {
      ledc_channel_config(&ledc_channel[ch]);
    }
  }

  // Initialize fade service. (none used...)
  ledc_fade_func_install(0);

  return 0;
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! set target position and move fader
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \param[in] pos motorfader position 
//!      (resolution depends on the used AIN resolution, usually 12bit)
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_FaderMove(uint32_t mf, uint16_t pos)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  // skip if fader currently manually moved (feedback killer)
  if( mf_state[mf].manual_move_ctr )
    return 0; // no error

  // skip if current position already inside MF deadband (we cannot do it better anyhow...)
  int32_t mf_delta = mf_state[mf].current_pos - pos;
  if( abs(mf_delta) < mf_state[mf].config.mf_deadband )
    return 0; // no error

  // following sequence must be atomic
  //MIOS32_IRQ_Disable();

  // set new motor position
  mf_state[mf].target_pos = pos;

  // reinit repeat and timeout counter
  mf_state[mf].repeat_ctr = REPEAT_CTR_RELOAD;
  mf_state[mf].timeout_ctr = TIMEOUT_CTR_RELOAD;
  mf_state[mf].duty_cycle = mf_state[mf].config.pwm_period;

  // reset trace buffer if armed and fader matches with trace fader
  if( mf_trace.state == MF_TRACE_STATE_ARMED && mf_trace.fader == mf ) {
    mf_trace.scale_ctr = 0;
    mf_trace.pos = 0;
    mf_trace.state = MF_TRACE_STATE_RUNNING;
  }
  
  //MIOS32_IRQ_Enable();

  return 0; // no error
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! direct control over the motorfader
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \param[in] direction is MF_Standby, MF_Up or MF_Down
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_FaderDirectMove(uint32_t mf, mfdrv_direction_t direction)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  // set new motor direction (must be atomic)
  //MIOS32_IRQ_Disable();
  mf_state[mf].direction = direction;
  mf_state[mf].direct_control = (direction == MF_Standby) ? 0 : 1;
  //MIOS32_IRQ_Enable();

  return 0; // no error
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! returns current position of a fader as a 12bit value
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_FaderPositionGet(uint32_t mf)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  return mf_state[mf].current_pos;
#endif
}

/////////////////////////////////////////////////////////////////////////////
//! requests a fader trace
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \param[in] scale record each n-1 sample
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_TraceRequest(uint32_t mf, uint8_t scale)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  if( mf_trace.state == MF_TRACE_STATE_RUNNING ) {
    mf_trace.scale_ctr = 0;
    mf_trace.pos = 0;
  } else {
    mf_trace.state = MF_TRACE_STATE_ARMED;
  }
  mf_trace.fader = mf;
  mf_trace.scale = scale;

  return 0; // no error
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! returns fader trace once available (only once! Next poll will return 0 again)
//! \param[in] trace pointer to trace data
//! \return 1 if trace available, otherwise 0
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_TraceAvailable(uint16_t **trace)
{
  *trace = &mf_trace.data[0];
  if( mf_trace.state == MF_TRACE_STATE_FINISHED ) {
    mf_trace.state = MF_TRACE_STATE_SENT;
    return 1;
  }
    
  return 0; // no trace available
}


/////////////////////////////////////////////////////////////////////////////
//! activates/deactivates suspend mode of motor<BR>
//! (function used by touchsensor detection)
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \param[in] suspend 1 to enable, 0 to disable suspend
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_SuspendSet(uint32_t mf, uint8_t suspend)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  if( suspend ) {
    mf_state[mf].suspended = 1;
  } else {
    mf_state[mf].suspended = 0;
    mf_state[mf].manual_move_ctr = MANUAL_MOVE_CTR_RELOAD;
  }

  return 0; // no error
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! returns the suspend state of the motor
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \return -1 if motor doesn't exist
//! \return 1 if motor suspended
//! \return 0 if motor not suspended
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_SuspendGet(uint32_t mf)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  return mf_state[mf].suspended;
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! this function resets the software implemented touch detection, so that the
//! fader is repositioned regardless if it is currently moved or not
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_TouchDetectionReset(uint32_t mf)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  mf_state[mf].manual_move_ctr = 0;

  return 0; // no error
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! this function returns the current state of the touch sensor
//! \param[in] ts touch sensor number (0..MFDRV_NUM-1)
//! \return 0 if sensor touched, 1 if sensor not touched
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_TouchSensorGet(uint32_t ts)
{
#if !MFDRV_NUM
  return 1; // no motors
#else
  // check if motor exists
  if( ts >= MFDRV_NUM )
    return 1;

  return mf_state[ts].current_ts_value;
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! This function configures various MF driver parameters.<BR>
//! see http://www.ucapps.de/mbhp_mf.html for detailed informations about
//! these parameters.
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \param[in] config a structure with following members:
//! <UL>
//!   <LI>mf_config.mf_deadband
//!   <LI>mf_config.pwm_period
//!   <LI>mf_config.pwm_duty_cycle_up_min/max
//!   <LI>mf_config.pwm_duty_cycle_down_min/max
//! </UL>
//! \return -1 if motor doesn't exist
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_ConfigSet(uint32_t mf, mfdrv_config_t config)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  // check if motor exists
  if( mf >= MFDRV_NUM )
    return -1;

  // take over new configuration
  mf_state[mf].config = config;

  return 0; // no error
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! returns the MF configuration
//! \param[in] mf motor number (0..MFDRV_NUM-1)
//! \return mf_config.mf_deadband
//! \return mf_config.pwm_period
//! \return mf_config.pwm_duty_cycle_up_min/max
//! \return mf_config.pwm_duty_cycle_down_min/max
/////////////////////////////////////////////////////////////////////////////
mfdrv_config_t MFDRV_ConfigGet(uint32_t mf)
{
  const mfdrv_config_t dummy = { };
#if !MFDRV_NUM
  return dummy;
#else
  // MF number valid?
  if( mf >= MFDRV_NUM ) {
    return dummy;
  }

  return mf_state[mf].config;
#endif
}


/////////////////////////////////////////////////////////////////////////////
//! Called from AIN DMA interrupt whenever new conversion results are available
//! \param[in] *ain_values pointer to current conversion results
//! \param[in] _callback_manual_fader_change callback on manual fader changes
//!             (can be used to send MIDI messages)
//! \return -1 on errors
/////////////////////////////////////////////////////////////////////////////
int32_t MFDRV_Tick(uint16_t *ain_values, uint8_t *dinsr_values, void *_callback_manual_fader_change, void *_callback_touch_sensor_change)
{
#if !MFDRV_NUM
  return -1; // no motors
#else
  void (*callback_manual_fader_change)(uint32_t mf, uint16_t value) = _callback_manual_fader_change;
  void (*callback_touch_sensor_change)(uint32_t ts, uint16_t value) = _callback_touch_sensor_change;
  int i;

  // check all motorfaders
  mf_state_t *mf = (mf_state_t *)&mf_state;
  for(i=0; i<MFDRV_NUM; ++i) {
    // skip if fader directly controlled
    if( !mf->direct_control ) {
      // AIN changes
      uint16_t current_pos = ain_values[i];
      uint16_t ain_delta = abs(current_pos - mf->current_pos);

      // store new position if outside deadband, otherwise take previous one
      if( ain_delta > mf->config.ain_deadband ) {
	mf->current_pos = current_pos;
      }

      // trace enabled?
      if( mf_trace.fader == i && mf_trace.state == MF_TRACE_STATE_RUNNING ) {
	if( ++mf_trace.scale_ctr >= mf_trace.scale ) {
	  mf_trace.scale_ctr = 0;

	  if( mf_trace.pos < 256 ) {
	    mf_trace.data[mf_trace.pos++] = current_pos;
	  } else {
	    mf_trace.state = MF_TRACE_STATE_FINISHED;
	  }
	}
      }

      // counter handling
      if( mf->manual_move_ctr )
	--mf->manual_move_ctr;
      if( mf->timeout_ctr )
	--mf->timeout_ctr;
      
      // Touch sensor changes
      {
	uint8_t ts_value = (dinsr_values[i/8] & (1 << (i % 8))) ? 1 : 0;
	if( ts_value != mf->current_ts_value ) {
	  mf->current_ts_value = ts_value;
	  callback_touch_sensor_change(i, ts_value);
	}
      }
      
      // check touch detection, shutdown motor if active
      if( mf->suspended ) {
        // no repeats
        mf->repeat_ctr = 0;
        // no timeout
        mf->timeout_ctr = 0;
      }
  
      // check motor state
      mf->idle = 0;
  
      // CASE: motor on target position?
      if( !mf->repeat_ctr ) {
        // motor should go into standby mode
        mf->direction = MF_Standby;
        mf->idle = 1;
  
        // timeout reached?
        if( mf->timeout_ctr ) {
	  // no: copy current AIN value into target position (reassurance phase)
	  mf->target_pos = current_pos;
        } else {
	  // AIN value was outside deadband?
	  if( ain_delta > mf->config.ain_deadband ) {
	    // set manual move counter, so that the motor won't be moved during this time
	    mf->manual_move_ctr = MANUAL_MOVE_CTR_RELOAD;
	    // change flag should not be cleared
	    callback_manual_fader_change(i, current_pos);
	  }
        }
      }
      // CASE: motor very slow or not moving
      else if( !mf->timeout_ctr && ain_delta <= mf->config.mf_deadband ) {
        // if timeout reached, turn motor into idle
	mf->direction = MF_Standby;
	mf->idle = 1;
	mf->repeat_ctr = 0;
      }
      // CASE: motor is moving fast
      else if( ain_delta > mf->config.mf_deadband ) {
        // fine: reload timeout counter
	// TK: disabled - problematic on noisy lines, and should only be required for slow motor pots...?
        //mf->timeout_ctr = TIMEOUT_CTR_RELOAD;
      }
      
      // continue if motor control hasn't reached idle state
      if( !mf->idle ) {
	
        // don't move motor if speed too fast
        if( ain_delta > 2000 ) { // TODO: check with panasonic faders
	  mf->direction = MF_Standby;
        } else {
	  // determine into which direction the motor should be moved
	  
	  // special cases: if target and current position <= min or > max value, stop motor
	  // (workaround for ALPS faders which never reach the 0x000 value)
	  if( (mf->target_pos < mf->config.fader_min_value && current_pos < mf->config.fader_min_value) ||
	      (mf->target_pos > mf->config.fader_max_value && current_pos > mf->config.fader_max_value) ) {
	    mf->direction = MF_Standby;
	    --mf->repeat_ctr;
	  } else {
	    // check distance between current and target position
	    // if fader is in between the MF deadband, don't move it anymore
	    int32_t mf_delta = current_pos - mf->target_pos;
	    uint32_t abs_mf_delta = abs(mf_delta);

	    // dynamic deadband: depends on repeat counter
	    uint8_t dyn_deadband;
	    if( mf->repeat_ctr < 4 )
	      dyn_deadband = 16;
	    else if( mf->repeat_ctr < 8 )
	      dyn_deadband = 32;
	    else if( mf->repeat_ctr < 16 )
	      dyn_deadband = 64;
	    else
	      dyn_deadband = mf->config.mf_deadband;

	    if( abs_mf_delta <= dyn_deadband ) {
	      mf->idle = 1;
	      --mf->repeat_ctr;
	    }
	    
	    // slow down motor via PWM if distance between current and target position < SLOWDOWN_THRESHOLD
	    if( mf->config.pwm_period && abs_mf_delta < SLOWDOWN_THRESHOLD ) {
	      // scale duty cycle depending on distance to target position
	      uint8_t duty_cycle_min = (mf_delta > 0) ? mf->config.pwm_duty_cycle_up_min : mf->config.pwm_duty_cycle_down_min;
	      uint8_t duty_cycle_max = (mf_delta > 0) ? mf->config.pwm_duty_cycle_up_max : mf->config.pwm_duty_cycle_down_max;

	      mf->duty_cycle = duty_cycle_min + ((abs_mf_delta * (duty_cycle_max-duty_cycle_min)) / SLOWDOWN_THRESHOLD);
	    } else {
	      // otherwise: full speed
	      mf->duty_cycle = mf->config.pwm_period;
	    }
	    
	    // check if motor should be moved up/down
	    if( mf->idle )
	      mf->direction = MF_Standby;
	    else {
	      mf->direction = (mf_delta > 0) ? MF_Up : MF_Down;
	    }
	  }
        }
      }
    }

    // finally update duty cycles
    {
      uint32_t ch0 = 2*i + 0;
      uint32_t ch1 = 2*i + 1;
      uint32_t duty_cycle_up = 0;
      uint32_t duty_cycle_down = 0;

      if( mf->config.fader_mode.inverted ) {
	uint32_t swap = ch0;
	ch0 = ch1;
	ch1 = swap;
      }

      if( mf->config.fader_mode.enabled ) {
	if( mf->direction == MF_Up )
	  duty_cycle_up = mf->duty_cycle;
	else if( mf->direction == MF_Down )
	  duty_cycle_down = mf->duty_cycle;
      }

      ledc_set_duty(ledc_channel[ch0].speed_mode, ledc_channel[ch0].channel, duty_cycle_up);
      ledc_update_duty(ledc_channel[ch0].speed_mode, ledc_channel[ch0].channel);

      ledc_set_duty(ledc_channel[ch1].speed_mode, ledc_channel[ch1].channel, duty_cycle_down);
      ledc_update_duty(ledc_channel[ch1].speed_mode, ledc_channel[ch1].channel);

    }
    
    // switch to next motorfader
    ++mf;
  }

  return 0; // no error
#endif
}

//! \}
