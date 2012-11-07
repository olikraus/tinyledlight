

#include <stdint.h>

#define ADC_RING_BUFFER_SIZE 6

#define STATE_SILENCE_WAIT			0
#define STATE_NOISE_DETECTED		1
#define STATE_CONFIRM_ON_1			2
#define STATE_CONFIRM_ON_2			3



/* ===== calibration variables ===== */

/* 
  threshold: amplitude above this will be detected as clap sound
  range: 0..2048 
*/
int16_t noise_intensity = 400;

/*
  maximum number of ticks for the clap noise. one tick is 1ms, clap noise is usually not longer than 20ms, noise_min_time < noise_max_time 
  range: 9..99
*/
uint16_t noise_max_time = 20;
/*
  minimum number of ticks for the clap noise. one tick is 1ms, clap noise is usually longer than 3ms, noise_min_time < noise_max_time
  range: 9..99
*/
uint16_t noise_min_time = 7;	 
/*
  number of ticks for flash on confirmation time, one tick is 1ms, should be 300 
  range: 1...999
*/
uint16_t confirm_on_flash_time = 300;
/*
  number of ticks to wait after flash on confirm time, one tick is 1ms, should be about 100
  range: 1...999
*/
uint16_t confirm_on_delay_time = 100;
/*
  initial pwm value for confirm off, 500
  range: 1...1023	
*/
uint16_t confirm_off_initial_pwm;
/*
  number of ticks for confirmation off, initial_pwm is decreased by 1 for each tick, 500
  range:1..1023		
*/
uint16_t confirm_off_time;


/* ===== signals/variables ===== */
int16_t adc_differential_value;
int16_t adc_amplitude;
uint8_t is_noise; /* 1, if adc_amplitude is above noise_intensity */
uint8_t state = STATE_SILENCE_WAIT;

/*
    0: led light is off
    1: led light is controlled by state machine
	  led value taken from led_direct_value
    2:	led light is controlled by user (potentiometer)
	  led value taken from led_target_value and led_current_value;
*/
#define LED_OFF 0
#define LED_STATE_MACHINE 1
#define LED_USER 2

uint8_t  led_control = LED_OFF;


uint16_t count;
int16_t adc_rb_mem[ADC_RING_BUFFER_SIZE];
uint8_t adc_rb_pos = 0;


/* ===== helper procedures ===== */

void cnt_reset(void)
{
  count = 0;
}

void cnt_inc(void)
{
  count++;
}



void adc_rb_init(void)
{
  uint8_t i;
  adc_rb_pos = 0;
  for( i = 0; i < ADC_RING_BUFFER_SIZE; i++ )
  {
    adc_rb_mem[i] = 0;
  }
}

void adc_rb_add(int16_t val)
{
  adc_rb_mem[adc_rb_pos] = val;
  adc_rb_pos++;
  if ( adc_rb_pos > ADC_RING_BUFFER_SIZE )
    adc_rb_pos = 0;
}

int16_t adc_rb_get_max(void)
{
  int16_t m;
  m = adc_rb_mem[0];
  for( i = 1; i < ADC_RING_BUFFER_SIZE; i++ )
  {
    if ( m < adc_rb_mem[i] )
      m = adc_rb_mem[i];
  }
  return m;
}

int16_t adc_rb_get_min(void)
{
  int16_t m;
  m = adc_rb_mem[0];
  for( i = 1; i < ADC_RING_BUFFER_SIZE; i++ )
  {
    if ( m > adc_rb_mem[i] )
      m = adc_rb_mem[i];
  }
  return m;
}

/* ===== signal processing ===== */

/*
  input: adc_differential_value
  output: is_noise
*/
void detect_noise(void)
{
  /* read signal adc_differential_value and put it into the ring buffer */
  adc_rb_add(adc_differential_value);
  
  /* calculate peak to peak distance for the adc values in the ring buffer */
  adc_amplitude = adc_rb_get_max() - adc_rb_get_min();
  
  /* check if we have a noise detected */
  is_noise = 0;
  if ( adc_amplitude > noise_intensity )
    is_noise = 1;
}



void state_machine(void)
{
  cnt_inc();
  switch(state)
  {
    case STATE_SILENCE_WAIT:
      if ( is_noise != 0 )
      {
	cnt_reset();
	state = STATE_NOISE_DETECTED;
      }
      break;
    case STATE_NOISE_DETECTED:
      if ( count > noise_max_time )
      {
	state = STATE_NOISE_WAIT;		/* noise too long */
      }
      else if ( is_noise == 0 && count < noise_min_time )
      {
	state = STATE_SILENCE_WAIT;		/* noise too short */
      }
      else if ( is_noise == 0 && led_control == LED_OFF )
      {
	state = STATE_CONFIRM_ON_1;		/* clap detected */
	cnt_reset();
	led_control = LED_STATE_MACHINE;
      }
      else if ( is_noise == 0 )
      {
	state = STATE_CONFIRM_OFF_1;		/* clap detected */
	cnt_reset();
	led_control = LED_STATE_MACHINE;
      }
    case STATE_NOISE_WAIT:
      if ( is_noise == 0 )
	state = STATE_SILENCE_WAIT;
      
      break;
    default:
      state = STATE_SILENCE_WAIT;
      break;
  }
}