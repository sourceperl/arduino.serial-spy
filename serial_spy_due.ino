// auto serial baudrate discover, store and display byte dump,
// modbus diagnostic tool
//
// target : Arduino DUE
// note   : - signal to detect must be wire on pin 19 (Rx of Serial1).
//          - use bitlash for console manager (see https://github.com/billroy/bitlash)
//          - use Timer for schedule tasks (see  https://github.com/JChristensen/Timer)

// library
#include "bitlash.h"
#include "Timer.h"

// macro
#define ROW_COUNT(array) (sizeof(array) / sizeof(*array))

// const
// pin 19 = interrupt 19 on DUE board
const byte SERIAL_IN_PIN = 19;
const byte SERIAL_IN_INT = 19;

// struct
struct in_bytes {
  byte value;
  unsigned long time;
  bool set;
  bool ahead;
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
unsigned long isr_micros;
unsigned long serial1_micros;
Timer t;

// ISR call when edges rising/falling occurs on SERIAL_IN_PIN
void isr_serial_change() {
  // update slots array
  if (isr_index_buffer > 59)
    isr_index_buffer = 0;
  isr_times_buffer[isr_index_buffer++] = micros() - isr_micros;
  isr_micros = micros();
}

void update_time_buffer(void) {
  // search minimal/maximal time step
  max_step = 0;
  min_step = 0xFFFF;
  noInterrupts();
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
  interrupts();
  // compute auto baudrate
  auto_baud = 1e6 / min_step;
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
  printf("Index |  Time (ms)  | Hex  | Dec | ASCII | mbus ahead\r\n");
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
      char ascii = ((rx_bytes[i].value >= 0x20) and (rx_bytes[i].value <= 0x7e)) ? rx_bytes[i].value : ' ';
      char mbus_ahead = (rx_bytes[i].ahead) ? 'x' : ' ';
      printf("  %3d | %11ld | 0x%02x | %3d |   %c   |     %c\r\n", line++, rx_bytes[i].time - t_origin, rx_bytes[i].value, rx_bytes[i].value, ascii, mbus_ahead);
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
  printf("\r\n");
  return 0;
}

// flush all datas in dump cache
numvar resetDump(void) {       
  for (word i = 0; i < ROW_COUNT(rx_bytes); i++) {
    rx_bytes[i].set = false; 
  }
  printf("data cache empty\r\n");
  return 0;
}

// display min_step time and compute baudrate
numvar printStat(void) {       
  printf("min step  : %5d us\r\n", min_step);
  printf("max step  : %5d us\r\n", max_step);
  return 0;
}

numvar setBaud(void) {
  uart_baud = getarg(1);
  Serial1.begin(uart_baud);
  printf("baud UART      : %6lu bps\r\n", uart_baud);
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
  printf("baud measured  : %6lu bps\r\n", auto_baud);
  printf("baud estimated : %6lu bps\r\n", norm_baud(auto_baud));
  printf("baud UART      : %6lu bps\r\n", uart_baud);
  return 0;  
}

void setup(void) {
  // vars init
  rx_byte_index = 0;
  isr_index_buffer = 0;
  // init bitlash
  initBitlash(115200);
  // add spy functions to bitlash function
  addBitlashFunction("spy_dump", (bitlash_function) printRaw);
  addBitlashFunction("spy_reset", (bitlash_function) resetDump);  
  addBitlashFunction("spy_stat", (bitlash_function) printStat);
  addBitlashFunction("spy_baud", (bitlash_function) printBaud);
  addBitlashFunction("spy_setbaud", (bitlash_function) setBaud);
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
    // micro time since last byte
    unsigned long time_last = micros() - serial1_micros;
    serial1_micros = micros();    
    // add rx byte to array
    rx_bytes[rx_byte_index].value = inChar;
    rx_bytes[rx_byte_index].time = millis();
    rx_bytes[rx_byte_index].set = true;    
    rx_bytes[rx_byte_index].ahead = (time_last >= (3.5 * 1e6/(uart_baud/10)));
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
