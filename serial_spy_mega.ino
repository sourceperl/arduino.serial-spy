// auto serial baudrate discover
//
// target : Arduino Mega (ATMega2560)
// note   : - use timer1 for 0.5 uS accuracy time measure 
//            (arduino core micros() is just 4us accuracy).
//          - signal to detect must be wire on pin 19 (Rx of Serial1).
//          - use bitlash for console manager (see https://github.com/billroy/bitlash)
//          - use Timer for schedule tasks (see  https://github.com/JChristensen/Timer)

// library
#include "bitlash.h"
#include "Timer.h"

// macro
#define ROW_COUNT(array)  (sizeof(array) / sizeof(*array))

// const
// pin 2 = interrupt 1 on UNO board
const byte SERIAL_IN_PIN = 19;
const byte SERIAL_IN_INT = 4;

// struct
struct in_bytes {
  byte value;
  unsigned long time;
  bool set;
};

// vars
in_bytes rx_bytes[512];
word rx_byte_index;
volatile word isr_times_buffer[60];
byte isr_index_buffer;
word max_step, min_step;
byte max_index, min_index;
unsigned long auto_baud;
unsigned long uart_baud;
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
  // search minimal/maximal time step
  max_step = 0;
  min_step = 0xFFFF;
  cli();
  for (byte i = 0; i < 60; i++) {
    // skip special values
    if ((isr_times_buffer[i] != 0) and (isr_times_buffer[i] != 0xFFFF))  {
      // search max step
      if (isr_times_buffer[i] > max_step) {
        max_step = isr_times_buffer[i];
        max_index = i;
      }    
      // search min step
      if (isr_times_buffer[i] < min_step) {
        min_step = isr_times_buffer[i];
        min_index = i;
      }
    }
  }
  sei();
  // compute auto baudrate
  auto_baud = 2 * (1e6 / min_step);
}

// display RAW data receive on UART Serial1
// arg1 is max number of line to print
// arg2 is lifo_mode (new value first)
numvar printRaw(void) {
  word disp_row = ROW_COUNT(rx_bytes);
  bool lifo_mode = false;
  // can force max row with arg
  if (getarg(0) > 0)
    disp_row = (getarg(1) < ROW_COUNT(rx_bytes)) ? getarg(1) : ROW_COUNT(rx_bytes);
  if (getarg(0) > 1)
    lifo_mode = (getarg(2) != 0);
  // banner
  printf_P(PSTR("Index |  Time (ms)  | Hex  | Dec | ASCII\r\n"));
  // datas
  word i = (lifo_mode and rx_byte_index) ? rx_byte_index - 1: rx_byte_index;
  word index = 0;
  word line = 0;
  unsigned long t_origin = 0;
  /* loop: 
  fifo : [rx_byte_index] (++) -> end array -> 0 (++) -> [rx_byte_index-1]
  lifo : [rx_byte_index-1] (--) -> 0 -> end array (--) -> [rx_byte_index] */
  while((index < ROW_COUNT(rx_bytes)) and (line < disp_row)) {
    if (rx_bytes[i].set) {
      // search first timestamp for time origin
      if (t_origin == 0)
        t_origin = rx_bytes[i].time;
      // no printable ascii -> set to 0x00
      char ascii = ((rx_bytes[i].value >= 0x20) and (rx_bytes[i].value <= 0x7e)) ? rx_bytes[i].value : 0x00;
      printf_P(PSTR("  %3d | %11ld | 0x%02x | %3d | %c\r\n"), line++, rx_bytes[i].time - t_origin, rx_bytes[i].value, rx_bytes[i].value, ascii);
    }
    index++;
    // next byte
    if (! lifo_mode) {
      // old data first 
      if (++i >= ROW_COUNT(rx_bytes))
        i = 0;
    } else {
      // new data first
      if (i-- == 0)
        i = ROW_COUNT(rx_bytes) - 1;
    }
  }
  printf_P(PSTR("\r\n"));
  return 0;
}

numvar printStat(void) {       
  // display min_step time and compute baudrate
  printf_P(PSTR("min step  : %5d us\r\n"), min_step/2);
  printf_P(PSTR("max step  : %5d us\r\n"), max_step/2);
  return 0;
}

numvar setBaud(void) {
  uart_baud = getarg(1);
  Serial1.begin(uart_baud);
  return 0;
}

// return true if theoric baudrate and measure baudrate are 10% accuracy
bool is_baud(long theoric, long measure) {
  return (measure > (theoric - theoric * 0.1)) and (measure < (theoric + theoric *0.1));
}

// convert "baud" (like 305) to normalized baud (like 300 or 9600)
unsigned long norm_baud(unsigned long baud) {
  if (is_baud(300, baud))
    return 300;
  else if (is_baud(600, baud))
    return 600;  
  else if (is_baud(1200, baud))
    return 1200;  
  else if (is_baud(2400, baud))
    return 2400;  
  else if (is_baud(4800, baud))
    return 4800;  
  else if (is_baud(9600, baud))
    return 9600;  
  else if (is_baud(14400, baud))
    return 14400;  
  else if (is_baud(19200, baud))
    return 19200;  
  else if (is_baud(28800, baud))
    return 28800;  
  else if (is_baud(38400, baud))
    return 38400;  
  else if (is_baud(57600, baud))
    return 57600;  
  else if (is_baud(115200, baud))
    return 115200;
  else 
    return 0;
}

numvar printBaud(void) {
  printf_P(PSTR("baud measured  : %6lu bps\r\n"), auto_baud);
  printf_P(PSTR("baud estimated : %6lu bps\r\n"), norm_baud(auto_baud));
  printf_P(PSTR("baud UART      : %6lu bps\r\n"), uart_baud);
  return 0;  
}

// link stdout (printf) to Serial object
// create a FILE structure to reference our UART and LCD output function
static FILE uartout = {0};

// create a output function
// This works because Serial.write, although of
// type virtual, already exists.
static int uart_putchar (char c, FILE *stream) {
  Serial.write(c);
  return 0;
}

void setup(void) {
  // vars init
  rx_byte_index = 0;
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
  // add spy functions to bitlash function
  addBitlashFunction("spy_dump", (bitlash_function) printRaw);
  addBitlashFunction("spy_stat", (bitlash_function) printStat);
  addBitlashFunction("spy_baud", (bitlash_function) printBaud);
  addBitlashFunction("spy_setbaud", (bitlash_function) setBaud);
  // fill in the UART file descriptor with pointer to writer
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  // standard output device STDOUT is uart
  stdout = &uartout ;
  // init serial 1 (pin18/19)
  Serial1.begin(9600);
  uart_baud = 9600;
  // launch timer task
  int updateJob = t.every(500, update_time_buffer);
  // IO init
  pinMode(SERIAL_IN_PIN, INPUT);
  attachInterrupt(SERIAL_IN_INT, isr_serial_change, CHANGE);
}

// call if data available on Serial1
void serialEvent1() {
  while (Serial1.available()) {
    // read rx byte
    char inChar = (char)Serial1.read();
    // add rx byte to array
    rx_bytes[rx_byte_index].value = inChar;
    rx_bytes[rx_byte_index].time = millis();
    rx_bytes[rx_byte_index].set = true;    
    if (++rx_byte_index >= ROW_COUNT(rx_bytes))
      rx_byte_index = 0;
  }
}

void loop(void) {
  // bitlash job
  runBitlash();
  // timer job
  t.update();
}
