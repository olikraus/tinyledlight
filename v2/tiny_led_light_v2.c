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
void setup_adc(void)
{
  DDRB &= ~_BV(4);              // use PB4/ADC2 as input pin for the ADC
  ADMUX = 2;	                        // Vcc as reference, ADC 2
  ADCSRB = 0x0;                         // default operation
  
}

uint16_t get_adc(void)
{
  uint16_t  l, h;
  
  ADCSRA = 0xc4;
  /* wait for conversion to be finished */
  while ( ADCSRA == 0xc4 )
    ;
  /* return 8 bit result */
  l = ADCL;
  h = ADCH;
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


/*=======================================================================*/
/*
  input: adc_value (10 bit)
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
  t = adc_value;
  t *= adc_value;
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

void setup_timer1_one_shot(void)
{
  DDRB |= 0x002;        // set OC1A (PB1) as output
  
  
  if ( duty == 0 || is_idle != 0 )
  {
    TCCR1 = 0x030;        // stop timer 2, set OC1A to logic one (MOSFET disabled) with the next command
    GTCCR |= _BV(2);        // OCR1B is not used, force OC1A (to logic one because of the previous command)
    
  }
  else
  {
    TCCR1 = 0x020;        // stop timer 2, set OC1A to logic zero (MOSFET enable) with the next command
    GTCCR |= _BV(2);        // OCR1B is not used, force OC1A (to logic one because of the previous command)
  
    TCNT1 = 0;               // counter value
    OCR1A = duty;
    TCCR1 = 0x037 - factor;        // clear OC1A after timeout  
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

/*=======================================================================*/
/* 2ms task: timer 0 overflow, with 500 Hz */

uint8_t pin_state = 0;

ISR(TIMER0_OVF_vect)
{
  TCNT0 = 255-125;			/* restart at 130 count up to 255 --> 1000 Hz */
  DDRB |= _BV(3);
  DDRB |= _BV(2);

  // PORTB |= _BV(2);
  
  if ( pin_state == 0 )
    pin_state = 1;
  else
    pin_state = 0;    
  
  if ( pin_state == 0 )
    PORTB &= ~_BV(3);
  else
    PORTB |= _BV(3);
  
  setup_timer1_one_shot();      
  
    raw_adc_value = get_adc();
    adc_value = low_pass(&adc_z, raw_adc_value, 3);
    //    adc_value = raw_adc_value;
    detect_idle_timeout();
    calculate_factor_and_len_2();
  
  // PORTB &= ~_BV(2);
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
