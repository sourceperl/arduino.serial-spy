// auto serial baudrate discover
//
// target : Arduino Mega (ATMega2560)
// note   : - use timer1 for 0.5 uS accuracy time measure 
//            (arduino core micros() is just 4us accuracy).
//          - signal to detect must be wire on pin 2.
//          - use bitlash for console manager (see https://github.com/billroy/bitlash)
//          - use Timer for schedule tasks (see  https://github.com/JChristensen/Timer)

// library
#include "bitlash.h"
#include "Timer.h"

// const
// pin 2 = interrupt 1 on UNO board
const byte SERIAL_IN_PIN = 2;
const byte SERIAL_IN_INT = 0;

// vars
word times_buffer[60];
byte level;
volatile word isr_times_buffer[60];
byte isr_index_buffer;
word start_bit;
word max_step, min_step;
byte max_index, min_index;
unsigned long baud;
Timer t;

// ISR call when edges rising/falling occurs on SERIAL_IN_PIN
void isr_serial_change() {
  // vars
  word time;
  // check timer1 overflow flag
  if (TIFR1 & 0x01) {
    // reset overflow flag and timer
    TIFR1 = TIFR1 | 0x01;
    // on overflow time is 0xFFFF
    time  = 0xFFFF;
  } else {
    // save time between two edges, reset timer
    time  = TCNT1;
  }
  // reset timer
  TCNT1 =  0;
  // update slots array
  if (isr_index_buffer > 59)
    isr_index_buffer = 0;
  isr_times_buffer[isr_index_buffer++] = time;
}

void update_time_buffer(void) {
  // copy ISR buffer to times array (avoid long ISRs stop)
  cli();
  for (byte i = 0; i < 60; i++) {
    times_buffer[i] = isr_times_buffer[i];
  }
  sei();

  // search minimal/maximal time step
  max_step = 0;
  min_step = 0xFFFF;
  start_bit = 0;
  level = 0;

  for (byte i = 0; i < 60; i++) {
    // skip special values
    if ((times_buffer[i] != 0) and (times_buffer[i] != 0xFFFF))  {
      // search max step
      if (times_buffer[i] > max_step) {
        max_step = times_buffer[i];
        max_index = i;
      }    
      // search min step
      if (times_buffer[i] < min_step) {
        min_step = times_buffer[i];
        min_index = i;
      }
    }
    if (times_buffer[i] != 0)
      level++;
  }
  
  // compute baudrate
  baud = 2 * (1e6 / min_step);
   
  // count start bit
  for (byte i = 0; i < 60; i++) {
    // count start bit
    if (times_buffer[i] > (min_step * 15)) {
      start_bit++;
    }
  }
}

numvar printRAW(void) {  
  // display array
  /*
  for (byte i = 0; i < 60; i++) {
    Serial.print(times_buffer[i]);
    Serial.print(" ");
  }
  Serial.println();*/
      
  // display min_step time and compute baudrate
  printf_P(PSTR("start bit : %5d\r\n"), start_bit);
  printf_P(PSTR("min step  : %5d us\r\n"), min_step/2);
  printf_P(PSTR("max step  : %5d us\r\n"), max_step/2);
  printf_P(PSTR("level     : %5d\r\n"), level);

  return 0;
}

bool is_baud(long theoric, long measure) {
  return (measure > (theoric - theoric * 0.1)) and (measure < (theoric + theoric *0.1));
}

numvar printBaud(void) {
  printf_P(PSTR("baud measured  : %6lu bps\r\n"), baud);
  // convert measured baud to estimated baud
  long estim_baud = 0;
  
  if (is_baud(300, baud))
    estim_baud = 300;
  else if (is_baud(600, baud))
    estim_baud = 600;  
  else if (is_baud(1200, baud))
    estim_baud = 1200;  
  else if (is_baud(2400, baud))
    estim_baud = 2400;  
  else if (is_baud(4800, baud))
    estim_baud = 4800;  
  else if (is_baud(9600, baud))
    estim_baud = 9600;  
  else if (is_baud(14400, baud))
    estim_baud = 14400;  
  else if (is_baud(19200, baud))
    estim_baud = 19200;  
  else if (is_baud(28800, baud))
    estim_baud = 28800;  
  else if (is_baud(38400, baud))
    estim_baud = 38400;  
  else if (is_baud(57600, baud))
    estim_baud = 57600;  
  else if (is_baud(115200, baud))
    estim_baud = 115200;  
  
  printf_P(PSTR("baud estimated : %6lu bps\r\n"), estim_baud);
  return 0;  
}

// link stdout (printf) to Serial object
// create a FILE structure to reference our UART and LCD output function
static FILE uartout = {0};

// create a output function
// This works because Serial.write, although of
// type virtual, already exists.
static int uart_putchar (char c, FILE *stream)
{
  Serial.write(c);
  return 0;
}

void setup(void) {
  // vars init
  isr_index_buffer = 0;
  // timer1 init
  // disable global interrupts
  cli(); 
  TCCR1A = 0;
  // set timer runs at clock speed/8 (= 2 MHz -> 0.5us/step)
  TCCR1B = (1 << CS11);
  // enable global interrupts
  sei();
  // init bitlash
  initBitlash(57600);
  // add bitlash function
  addBitlashFunction("raw", (bitlash_function) printRAW);
  addBitlashFunction("bd", (bitlash_function) printBaud);
  // fill in the UART file descriptor with pointer to writer
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  // standard output device STDOUT is uart
  stdout = &uartout ;
  // launch timer task
  int updateJob = t.every(500, update_time_buffer);
  // IO init
  pinMode(SERIAL_IN_PIN, INPUT);
  attachInterrupt(SERIAL_IN_INT, isr_serial_change, CHANGE);
}

void loop(void) {
  // bitlash job
  runBitlash();
  // timer job
  t.update();
}
