/*

  tiny_led_light_v2.c
  
  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  Features
  - clap control 
  - 1000 Hz Interrupt
  
  Usage
    Light is switched on by clap noise, small flash feedback by light
    Light is switched off by very loud clap noise or by reaching idle time (default 1h)
    Variable resistor will always put light to the desired level
  
*/

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <util/delay.h>

/*=======================================================================*/
uint16_t raw_adc_value;
uint16_t adc_z = 0;
uint16_t adc_value;

uint8_t factor;
uint8_t duty;

uint8_t idle_detect_adc;
uint32_t idle_cnt = 0;
uint8_t is_idle = 0;

/* shutdown mosfet after 1h */
#define IDLE_MODE_MINUTES 60L




#define ADC_RING_BUFFER_SIZE 10

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
  disallow switch off light by noise
  range: 0, 1
*/
uint8_t is_light_off_with_noise = 1;

/* 
  threshold: amplitude above this will be detected as clap sound
  range: 0..2048 
  
  < 200 very easy to clap, large distance
  > 500 near distance, loud noise required
  
  
  light_on_noise_intensity		value to switch on the light
  light_off_noise_intensity		value to switch off the light
*/
int16_t light_on_noise_intensity = 150;
int16_t light_off_noise_intensity = 900;

/*
  maximum number of ticks for the clap noise. one tick is 1ms, clap noise is usually not longer than 20ms, noise_min_time < noise_max_time 
  This value MUST be larger than ADC_RING_BUFFER_SIZE + 1
  range: ADC_RING_BUFFER_SIZE + 1 .. 999
*/
uint16_t noise_max_time = 150;
/*
  minimum number of ticks for the clap noise. one tick is 1ms, clap noise is usually longer than 3ms, noise_min_time < noise_max_time
  range: 1..99
*/
uint16_t noise_min_time = 4;
/*
  confirmation light intensity 
  range: 1...1023
*/
uint16_t confirm_on_flash_lt = 1023;
/*
  number of ticks for flash on confirmation time, one tick is 1ms, should be 300 
  range: 1...999
*/
uint16_t confirm_on_flash_time = 100;
/*
  idle time after which the light is turned off, value in ticks (one minute are 60*1000 ticks)
  range:1..2^32-1
*/
uint32_t idle_time = 60UL*60UL*1000UL;
//uint32_t idle_time = 10*1000UL;
/*
  increment/decrement value for lt
  range:1..16, default is 4, should be power of 2
*/
uint16_t lt_step = 4;	


/* ===== signals/variables ===== */
int16_t noise_intensity = 600;
int16_t adc_diff_val;
uint8_t nd; /* 1, if adc_amplitude is above light_on_noise_intensity */
uint8_t pm;	/* 0: no var pot movement, 1: var pot has changed, auto reset to 0 by state machine */
uint16_t lt;		/* current light value 10 bit */
uint16_t pot;	/* var potentiometer position 10 bit*/
uint16_t pot_old_value;	/* var potentiometer position 10 bit*/



/*=======================================================================*/

void setup_system_clock(void)
{
  cli();        // disable interrupts
  CLKPR = 0x080;        // enable system clock programming
  CLKPR = 0x000;        // force division by 1 for the system clock, --> 8Mhz synchron clock  
  sei();        // enable interrupts
}

void delay_milli_seconds(uint8_t x)
{
  _delay_loop_2( (uint16_t)((unsigned long)x*(unsigned long)(F_CPU/4000UL)) );
}

/*=======================================================================*/
/* ADC */

#define ADC_PRESCALAR 0x07

void setup_adc(void)
{
}

/*
  measure voltage difference between ADC3 (positive, PORT B/Pin 4) and ADC2 (negative, PORT B/Pin 3)
  gain_bit == 0: 1x
  gain_bit == 1: 20x
*/
int16_t get_audio_adc(uint8_t gain_bit)
{
  uint16_t l, h, val;
  uint8_t sign;
  int16_t sval;
  
  /* datasheet recomends to turn off ADC for differntial measurement first */
  ADCSRA = 0x00 | ADC_PRESCALAR;		/* turn off ADC */  

  /* use PB3 and PB4 as input  */
  DDRB &= ~_BV(3);
  DDRB &= ~_BV(4);

  /* enable, but do not start ADC (ADEN, Bit 7) */
  /* clear the interrupt indicator flag (ADIF, Bit 4) */
  ADCSRA = 0x90 | ADC_PRESCALAR;
  
  ADMUX = 6 | gain_bit;
  /* enable bipolar mode, voltage diff may be higher or lower, result is signed */
  ADCSRB = 0x080;
  /* enable and start conversion  */
  ADCSRA = 0xc0|ADC_PRESCALAR;
  /* wait for conversion to be finished (ADIF, Bit 4) */
  while ( (ADCSRA & _BV(4)) == 0 )
    ;
  /* return 8 bit result */
  l = ADCL;
  h = ADCH;
  
  /* save some power */
  ADCSRA = 0x00 | ADC_PRESCALAR;		/* turn off ADC */  
  
  sign = 0;
  val = (h<<8) | l ;
  if ( val >= 512 )
  {
    sign = 1;
    val = 1024-val ;
  }
  
  sval = val;
  if ( sign != 0 )
    sval = -sval;
  
  
  return sval;  
}


/* variable potentiometer ADC read */
uint16_t get_variable_pot_adc(void)
{
  uint16_t l, h;

  /* turn off ADC to force long conversion */
  ADCSRA = 0x00 | ADC_PRESCALAR;		/* turn off ADC */  

  /* use PB2/ADC1 as input pin for the ADC */
  DDRB &= ~_BV(2);              // use PB2/ADC1 as input pin for the ADC
  PORTB &= ~_BV(2);		// switch off pull-up
  
  /* enable, but do not start ADC (ADEN, Bit 7) */
  /* clear the interrupt indicator flag (ADIF, Bit 4) */
  ADCSRA = 0x90 | ADC_PRESCALAR;
  
  /*  Vcc as reference, ADC 1 */
  ADMUX = 1;	
  /* default operation */
  ADCSRB = 0x0;
  /* enable and start conversion, maximum prescalar */
  ADCSRA = 0xc0|ADC_PRESCALAR;
  /* wait for conversion to be finished (ADIF, Bit 4) */
  while ( (ADCSRA & _BV(4)) == 0 )
    ;
  /* return 8 bit result */
  
  l = ADCL;
  h = ADCH;

  /* save some power */
  ADCSRA = 0x00 | ADC_PRESCALAR;		/* turn off ADC */  
  
  return (h<<8) | l ;
}


/*=======================================================================*/
/* low pass filter with 5 bit resolution, p = 0..31 */
uint16_t low_pass(uint16_t *a, uint16_t x, uint8_t p)
{
  uint16_t n;
  n = (32-p) * (*a) + p * x;
  n >>= 5;
  *a = n;
  return n;
}




/* ===== detect_noise ===== */

int16_t adc_rb_mem[ADC_RING_BUFFER_SIZE];
uint8_t adc_rb_pos = 0;
int16_t adc_amplitude;


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
  uint8_t i;
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
  uint8_t i;
  int16_t m;
  m = adc_rb_mem[0];
  for( i = 1; i < ADC_RING_BUFFER_SIZE; i++ )
  {
    if ( m > adc_rb_mem[i] )
      m = adc_rb_mem[i];
  }
  return m;
}

/*
  Input: 
    noise_intensity
    adc_diff_val
  Out: 
    nd	noise detection flag
  Internal Variables
    adc_amplitude
  Configuration Variables:
*/
void detect_noise(void)
{
  /* read signal adc_diff_val and put it into the ring buffer */
  adc_rb_add(adc_diff_val);
  
  /* calculate peak to peak distance for the adc values in the ring buffer */
  adc_amplitude = adc_rb_get_max() - adc_rb_get_min();
  
  /* check if we have a noise detected */
  nd = 0;
  if ( adc_amplitude > noise_intensity )
    nd = 1;
}


/* ===== state_machine ===== */

uint8_t state = STATE_LIGHT_OFF;
uint16_t cnt = 0;
uint32_t lcnt = 0;


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


/*
  In: 
    pot	variable pot value (0..1023)
    pm	variable pot moved
    nd	noise detection flag
  Out:
    lt		light value (0..1023)
  Internal Variables
    state
    cnt
    lcnt
  Configuration Variables:
    noise_min_time
    noise_max_time
    confirm_on_flash_lt
    confirm_on_flash_time
*/
void state_machine(void)
{
  switch(state)
  {
    case STATE_LIGHT_OFF:
      noise_intensity = light_on_noise_intensity;
      if ( pm != 0 )
      {
	state = STATE_INCREMENT_LIGHT;
      }
      else if ( nd != 0 )
      {
	cnt_zero();
	state = STATE_LIGHT_OFF_NOISE;
      }
      break;
      
    case STATE_LIGHT_OFF_NOISE:
      noise_intensity = light_on_noise_intensity;
      if ( pm != 0 )
      {
	state = STATE_INCREMENT_LIGHT;
      }
      else if ( nd == 0 && cnt < noise_min_time )
      {
	state = STATE_LIGHT_OFF;
      }
      else if ( nd == 0 && noise_min_time <= cnt && cnt <= noise_max_time )
      {
	lt = confirm_on_flash_lt;
	state = STATE_LIGHT_OFF_FLASH;
      }
      else if ( /* nd == 1 && */ cnt > noise_max_time )
      {
	state = STATE_LIGHT_OFF_WAIT;	
      }
      break;
      
    case STATE_LIGHT_OFF_WAIT:
      noise_intensity = light_on_noise_intensity;
      if ( pm != 0 )
      {
	state = STATE_INCREMENT_LIGHT;
      }
      else if ( nd == 0 )
      {
	state = STATE_LIGHT_OFF;
      }
      break;
      
    case STATE_LIGHT_OFF_FLASH:
      noise_intensity = light_on_noise_intensity;
      if ( pm != 0 )
      {
	state = STATE_INCREMENT_LIGHT;
      }
      else if ( cnt >= confirm_on_flash_time )
      {
	lt = 0;
	state = STATE_INCREMENT_LIGHT;	
      }
      break;
      
    case STATE_INCREMENT_LIGHT:
      noise_intensity = light_off_noise_intensity;
      if ( pot == 0 )
      {
	lt = 0;
	state = STATE_LIGHT_OFF;
      }
      else if ( nd != 0 )
      {
	lt = pot;
	cnt_zero();
	lcnt_zero();
	state = STATE_LIGHT_ON_NOISE;
      }
      else if ( lt >= pot )
      {
	lt = pot;
	lcnt_zero();
	state = STATE_LIGHT_ON;
      }
      else
      {
	lt+=lt_step;
      }
      break;
      
    case STATE_LIGHT_ON:
      noise_intensity = light_off_noise_intensity;
      if ( pm != 0 )
      {
	/* stay in this state */
	lt = pot;
      }
      else if ( lcnt >= idle_time )
      {
	state = STATE_DECREMENT_LIGHT;
      }
      else if ( nd == 1 && is_light_off_with_noise != 0 )
      {
	cnt_zero();
	state = STATE_LIGHT_ON_NOISE;
      }
      else
      {
	lt = pot;
      }
      break;
      
    case STATE_LIGHT_ON_NOISE:
      noise_intensity = light_off_noise_intensity;
      if ( pm == 1 )
      {
	/* lt = pot; */ /* done below */
	state = STATE_LIGHT_ON;
      }
      else if ( nd == 0 && cnt < noise_min_time )
      {
	state = STATE_LIGHT_ON;
      }
      else if ( nd == 0 && noise_min_time <= cnt && cnt <= noise_max_time )
      {
	state = STATE_DECREMENT_LIGHT;
      }
      else if ( /* nd == 1 && */ cnt > noise_max_time )
      {
	state = STATE_LIGHT_ON_WAIT;	
      }
      lt = pot;
      break;
      
    case STATE_LIGHT_ON_WAIT:
      noise_intensity = light_off_noise_intensity;
      if ( pm != 0 )
      {
	/* stay here and copy pot value (see below), wait until noise goes away */
      }
      /* else */ /* no else in this case, always leave this state if noise is away */
      if ( nd == 0 )
      {
	state = STATE_LIGHT_ON;
      }
      lt = pot;
      break;
      
    case STATE_DECREMENT_LIGHT:
      noise_intensity = light_on_noise_intensity;
      if ( pm != 0 )
      {
	state = STATE_INCREMENT_LIGHT;
      }
      else if ( lt == 0 )
      {
	state = STATE_LIGHT_OFF;
      }
      else
      {
	if ( lt > lt_step )
	  lt -= lt_step;
	else
	  lt = 0;
      }
      break;
      
    default:
      noise_intensity = light_on_noise_intensity;
      state = STATE_LIGHT_OFF;
      break;
  }
    
  cnt_inc();
  lcnt_inc();
  pm = 0;		/* reset var pot moved flag */
}


/*=======================================================================*/
/*
  input: lt (10 bit)
  output: factor, duty

  duty = 0...255  
  factor = 0...6

  use a quadratic equation

  y = m * adc_value  * adc_value
  where m = 1/64

*/


void calculate_factor_and_len_2(void)
{
  uint32_t t;
  uint16_t y; 
  t = lt;
  t *= lt;
  y = t >> 6;
  
  if ( y < 256 )
  {
    factor = 6;
    duty = y;
  }
  else
  {

    /*
    y = 256 --> factor = 5, duty = 128
    y = 512 --> factor = 4, duty = 128
    y = 1024 --> factor = 3, duty = 128
    y = 2048 --> factor = 2
    y = 4096 --> factor = 1
    y = 8192 --> factor = 0
    */
    
    factor = 6;
    while( y >= 256 )
    {
      factor--;
      y >>= 1;      
    }
    duty = y;
  }
}

/*=======================================================================*/
/* activate MOSFET for some time */

/*
  Input:
    is_idle:	0: normal operation, 1: disable all
    duty
    factor
*/
void setup_timer1_one_shot(void)
{
  DDRB |= 0x002;        // set OC1A (PB1) as output
  
  
  if ( duty == 0 || is_idle != 0 )
  {
    // TCCR1 = 0x030;        // V1: stop timer 2, set OC1A to logic one (MOSFET disabled) with the next command
    TCCR1 = 0x020;        // V2: stop timer 2, set OC1A to logic zero (MOSFET disabled) with the next command
    GTCCR |= _BV(2);        // OCR1B is not used, force OC1A (to samle logic level as before)
    
  }
  else
  {
    //TCCR1 = 0x020;        // stop timer 2, set OC1A to logic zero (MOSFET enable) with the next command
    TCCR1 = 0x030;        // V2: stop timer 2, set OC1A to logic one (MOSFET enable) with the next command
    GTCCR |= _BV(2);        // OCR1B is not used, force OC1A (to samle logic level as before)
  
    TCNT1 = 0;               // counter value
    OCR1A = duty;
    //TCCR1 = 0x037 - factor;        // V1: clear OC1A after timeout  
    TCCR1 = 0x027 - factor;        // V2: set OC1A after timeout  
  }
}


/*=======================================================================*/
void detect_idle_timeout(void)
{
  uint8_t x = adc_value >> 2;
  if (  idle_detect_adc != x )
  {
    idle_detect_adc = x;
    idle_cnt = 0;
    is_idle = 0;
  }
  else
  {
    if ( idle_cnt > 500L*60L*IDLE_MODE_MINUTES )
    {
      is_idle = 1;      
    }
    else
    {
      idle_cnt++;
    }
  }
}


void task_1ms(void)
{
  adc_diff_val = get_audio_adc(1);
 
  /* in: adc_diff_val */
  /* out: nd */
  detect_noise();
  
  /* in: pot, pm, nd */
  /* out: lt */
  state_machine();
  

// bypass state machine and output noise detection signal
/*  
  if ( nd == 0 )
    lt = 0;
  else
    lt = 1023;
*/
  
  /* in: lt */
  /* out: duty, factor */
  calculate_factor_and_len_2();
}

void task_2ms(void)
{
  /* in: duty, facter */
  /* out: - */
  setup_timer1_one_shot();      
  
  raw_adc_value = get_variable_pot_adc();  
  pot = low_pass(&adc_z, raw_adc_value, 3);
  if ( pot_old_value != pot )
    pm = 1;
  pot_old_value = pot;

}

/*=======================================================================*/
/* 1ms task: timer 0 overflow, with 500 Hz */

uint8_t pin_state = 0;
uint8_t div2 = 0;

ISR(TIMER0_OVF_vect)
{
  TCNT0 = 255-124;			/* restart at 130 count up to 255 --> 1000 Hz */
  DDRB |= _BV(0);
  //DDRB |= _BV(0);

  // PORTB |= _BV(0);
  
  if ( pin_state == 0 )
    pin_state = 1;
  else
    pin_state = 0;    
  
  if ( pin_state == 0 )
    PORTB &= ~_BV(0);
  else
    PORTB |= _BV(0);

  if ( div2 == 0 )
  {
    div2++;
  }
  else
  {
    div2 = 0;
    task_2ms();
  }
  
  task_1ms();
  
}


/*
  Timer 0 setup:
  - free-running
  - generates interrupt on overflow
  - overflow after 256 counts
  - prescalar 64
  - 8Mhz / (256*64) = 488 Hz
  - 8Mhz / ((256-125)*64) = 1000 Hz
*/
void setup_timer0_overflow(void)
{
  GTCCR = 0;        // OCR1B is not used, disable all, this is also used for timer 1 setup
  TCCR0A = 0;           // all external pins disconnected, normal mode
  TCCR0B = 3;           // devide by 64
  TIMSK |= _BV(1);      // generate interrupt on overflow
}



/*=======================================================================*/

int main(void)
{
  setup_system_clock();
  setup_adc();
  setup_timer0_overflow();
  
  for(;;)
  {
    set_sleep_mode(SLEEP_MODE_IDLE);  
    sleep_mode();
  }
  
  return 0;
}
