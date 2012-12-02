

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
  confirmation light intensity 
  range: 1...255
*/
uint16_t confirm_on_flash_lt = 255;
/*
  number of ticks for flash on confirmation time, one tick is 1ms, should be 300 
  range: 1...999
*/
uint16_t confirm_on_flash_time = 300;
/*
  number of ticks to wait after flash on confirm time, one tick is 1ms, should be about 100
  range: 1...999
*/
/* uint16_t confirm_on_delay_time = 100; */
/*
  initial pwm value for confirm off, 500
  range: 1...1023	
*/
/*uint16_t confirm_off_initial_pwm; */
/*
  number of ticks for confirmation off, initial_pwm is decreased by 1 for each tick, 500
  range:1..1023		
*/
/* uint16_t confirm_off_time; */
/*
  idle time after which the light is turned off, value in ticks
  range:1..2^32-1
*/
uint32_t idle_time = 60*60*1000;

/*
  increment/decrement value for lt
  range:1..16, default is 4, should be power of 2
*/
uint16_t lt_step = 4;	


/* ===== signals/variables ===== */
int16_t adc_diff_val;
uint8_t nd; /* 1, if adc_amplitude is above noise_intensity */
uint8_t pm;	/* 0: no var pot movement, 1: var pot has changed, auto reset to 0 by state machine */
uint16_t lt;		/* current light value 10 bit */
uint16_t pot;	/* var potentiometer position 10 bit*/




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
    adc_diff_val
  Out: 
    nd	noise detection flag
  Internal Variables
    adc_amplitude
  Configuration Variables:
    noise_intensity
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
    pot	variable pot value (0,,255)
    pm	variable pot moved
    nd	noise detection flag
  Out:
    lt		light value (0,,255)
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
      if ( pm != 0 )
      {
	/* stay in this state */
	lt = pot;
      }
      else if ( lcnt >= idle_time )
      {
	state = STATE_DECREMENT_LIGHT;
      }
      else if ( nd == 1 )
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
      state = STATE_LIGHT_OFF;
      break;
  }
    
  cnt_inc();
  lcnt_inc();
  pm = 0;		/* reset var pot moved flag */
}

/* ===== unix console ===== */

#ifdef __unix
#endif

#include <stdio.h>
#include <stdlib.h>

const char *get_state(void)
{
  switch(state)
  {
    case STATE_LIGHT_OFF: return "LIGHT_OFF";
    case STATE_LIGHT_OFF_NOISE: return "LIGHT_OFF_NOISE";    
    case STATE_LIGHT_OFF_WAIT: return "LIGHT_OFF_WAIT";
    case STATE_LIGHT_OFF_FLASH: return "LIGHT_OFF_FLASH"; 
    case STATE_INCREMENT_LIGHT: return "INCREMENT_LIGHT";
    case STATE_LIGHT_ON: return "LIGHT_ON";
    case STATE_LIGHT_ON_NOISE: return "LIGHT_ON_NOISE";
    case STATE_LIGHT_ON_WAIT: return "LIGHT_ON_WAIT";
    case STATE_DECREMENT_LIGHT: return "DECREMENT_LIGHT";
  }
  return "<UNKNOWN>";
}

void show_variables(void)
{
  printf("nd:%d pm:%d pot:%03d    cnt:%03d   lt:%03d    state:%s\n", nd, pm, pot, cnt, lt, get_state());
}

void show_config(void)
{
  printf("noise_min_time=%d\n",noise_min_time);
  printf("noise_max_time=%d\n",noise_max_time);
  printf("confirm_on_flash_lt=%d\n", confirm_on_flash_lt);
  printf("confirm_on_flash_time=%d", confirm_on_flash_time);
  puts("");
}

void space(const char **s)
{
  for(;;)
  {
    if ( **s == '\0' )
      return;
    if ( **s > 32 )
      return;
    (*s)++;
  }
}

int get_int(const char **s) 
{
  int x = 0;
  for(;;)
  {
    if ( **s < '0' || **s > '9' )
      break;
    x = x*10 + (((int)**s) - '0' );
    (*s)++;    
  }
  return x;
}


void exec(void)
{
  static char buf[1024];
  int arg;
  uint8_t old_state;
  const char *ptr = buf;
  uint8_t cmd;
  printf("> ");
  gets(buf);
  cmd = *ptr++;
  switch(cmd)
  {
    case 'h':
      puts("h: Help");
      puts("q: Quit");
      puts("c: show Config");
      puts("s <num>: do <num> steps, stop on state change");
      puts("n <num>: set Noise detect flag, <num> is 0 or 1");
      puts("p <num>: set Potentiometer value and pm to 1");
      puts("<return>: Execute state machine once");
      break;
    case 'q':
      exit(0);
      break;
    case 'c':
      show_config();
      break;
    case 's':
      space(&ptr);
      arg = get_int(&ptr);
      old_state = state;
      while( arg > 0 && old_state == state)
      {
	state_machine();
	pm = 0;
      }
      break;
    case 'n':
      space(&ptr);
      arg = get_int(&ptr);
      nd = arg;
      break;
    case 'p':
      space(&ptr);
      arg = get_int(&ptr);
      pot = arg;
      pm = 1;
      break;
    case '\0':
      state_machine();
      pm = 0;
      break;
  }
}

int main(void)
{
  for(;;)
  {
    show_variables();
    exec();
    
  }
  return 0;
}

