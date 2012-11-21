

#include <stdint.h>

#define ADC_RING_BUFFER_SIZE 6

#define STATE_LIGHT_OFF 0
#define STATE_LIGHT_OFF_NOISE 1
#define STATE_LIGHT_OFF_WAIT 2
#define STATE_LIGHT_OFF_FLASH 3
#define STATE_INCREMENT_LIGHT 4
#define STATE_LIGHT_ON 5
#define STATE_LIGHT_ON_NOISE 6
#define STATE_LIGHT_ON_WAIT 7
#define STATE_DECREMENT_LIGHT 8


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
uint8_t nd; /* 1, if adc_amplitude is above noise_intensity */
uint8_t lt;		/* current light value */
uint8_t state = STATE_LIGHT_OFF;




uint16_t cnt;
uint16_t lcnt;
int16_t adc_rb_mem[ADC_RING_BUFFER_SIZE];
uint8_t adc_rb_pos = 0;


/* ===== helper procedures ===== */

void cnt_zero(void)
{
  cnt = 0;
}

void cnt_inc(void)
{
  cnt++;
}

void lcnt_zero(void)
{
  lcnt = 0;
}

void lcnt_inc(void)
{
  lcnt++;
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
  output: nd
*/
void detect_noise(void)
{
  /* read signal adc_differential_value and put it into the ring buffer */
  adc_rb_add(adc_differential_value);
  
  /* calculate peak to peak distance for the adc values in the ring buffer */
  adc_amplitude = adc_rb_get_max() - adc_rb_get_min();
  
  /* check if we have a noise detected */
  nd = 0;
  if ( adc_amplitude > noise_intensity )
    nd = 1;
}



void state_machine(void)
{
  switch(state)
  {
    case STATE_LIGHT_OFF:
      if ( nd != 0 )
      {
	cnt_zero();
	state = STATE_LIGHT_OFF_NOISE;
      }
      if ( pm != 0 )
      {
	state = STATE_INCREMENT_LIGHT;
      }
      break;
    case STATE_LIGHT_OFF_NOISE:
      
      break;
    case STATE_LIGHT_OFF_WAIT:
      break;
    case STATE_LIGHT_OFF_FLASH:
      break;
    case STATE_INCREMENT_LIGHT:
      break;
    case STATE_LIGHT_ON:
      break;
    case STATE_LIGHT_ON_NOISE:
      break;
    case STATE_LIGHT_ON_WAIT:
      break;
    case STATE_DECREMENT_LIGHT:
      break;
    default:
      state = STATE_LIGHT_OFF;
      break;
  }
  cnt_inc();

}