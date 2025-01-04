// Test program for M48T59Y

#include <avr/pgmspace.h>

// Purpose of the test program is to verify that the
// functions of the M48T59Y are working correctly.

// There are two 74HC595 shift registers controlled by Arduino pins
// D2, D3, and D4. They are cascaded, and their outputs generate
// A0..A12 sent to the M48T59Y as its 13-bit address input.

// The first 74HC595 generates M48T59Y A0..A7, the second
// 74HC595 generates M48T59Y A8..A12. So, when an address is
// generated, it is shifted out in order from MSB (first)
// to LSB (last).

// Connections:
//   Arduino D2: 74HC595 serial data, pin 14 on first 74HC595
//   Arduino D3: 74HC595 serial clock, pin 11 on both 74HC595s
//   Arduino D4: 74HC595 register clock, pin 12 on both 74HC595s
//
//   Arduino D5: -W (active low write) signal to pin 27 on M48T59Y
//   Arduino D6: -G (active low output enable) to pin 22 on M48T59Y
//   Arduino D7: -E (active low chip enable) signal to pin 20 on M48T59Y
//
//   Arduino A0..A3 (controlled as low nybble of AVR PORTC):
//      connected to D0..D3 on the M48T59Y (its low 4 data bus pins)
//   Arduino D8..D11 (controlled as low nybble of AVR PORTB):
//      connected to D4..D7 on the M49T59Y (its high 4 data bus pins)
//
//   Arduino A4: connected to GO pushbutton (shorts to ground when pressed)
//   Arduino A5: connected to LOG pushbutton (shorts to ground when pressed)

#define SHIFTREG_DATA 2
#define SHIFTREG_SCLK 3
#define SHIFTREG_RCLK 4

#define M48T59Y_NW 5
#define M48T59Y_NG 6
#define M48T59Y_NE 7

#define GO  0x10  // pin 4 of PORTC
#define LOG 0x20  // pin 5 of PORTC

// All locations with addressess less than 1FF0h are memory locations.
#define M48T59Y_FIRST_REG 0x1FF0

// register addresses
#define M48T59Y_CONTROL 0x1FF8
#define M48T59Y_SECONDS 0x1FF9
#define M48T59Y_MINUTES 0x1FFA
#define M48T59Y_HOUR    0x1FFB
#define M48T59Y_DAY     0x1FFC
#define M48T59Y_DATE    0x1FFD
#define M48T59Y_MONTH   0x1FFE
#define M48T59Y_YEAR    0x1FFF

// For very short delays
#define NOP() __asm__ __volatile__ ("nop\n\t")

void assert_addr(uint16_t addr) {
  for (uint8_t i = 0; i < 16; ++i) {
    // assert one bit of address
    digitalWrite( SHIFTREG_DATA, (addr & 0x8000) != 0 );

    // clock in the bit
    digitalWrite( SHIFTREG_SCLK, 1 );
    digitalWrite( SHIFTREG_SCLK, 0 );

    // continue with remaining bits
    addr <<= 1;
  }

  // clock serial data into the register
  digitalWrite( SHIFTREG_RCLK, 1 );
  digitalWrite( SHIFTREG_RCLK, 0 );
}

/*
// We use an pseudo-random number generator
// to generate data to store in the M48T59Y's memory.
// Because the sequence is same as long as the seed is the
// same, we can store the data and then know how to verify it.
// By doing a verify pass *before* storing data, then we
// are testing whether the data contents are actually
// nonvolatile.

// https://stackoverflow.com/questions/17035441/looking-for-decent-quality-prng-with-only-32-bits-of-state

#define SEED 0

uint32_t state;
uint32_t rngData; // bytes of data available
uint8_t rngNumAvail; // how many bytes of data are available

uint32_t mulberry32(void) {
  uint32_t z = (state += 0x6D2B79F5);
  z = (z ^ z >> 15) * (1 | z);
  z ^= z + (z ^ z >> 7) * (61 | z);
  return z ^ z >> 14;
}

void rng_reset() {
  state = SEED;
  rngNumAvail = 0;
}

uint8_t rng_gen() {
  if ( rngNumAvail == 0 ) {
    rngData = mulberry32();
    rngNumAvail = 4;
  }
  uint8_t result = rngData & 0xFF;
  rngData >>= 8;
  --rngNumAvail;
  return result;
}
*/

// This boolean will keep track of whether the data bus
// pins are configured for reading or writing.
bool bus_reading;

// Set up DDRC and DDRB for reading from the M48T59Y.
void configure_data_bus_for_read() {
  DDRC = 0x00;
  DDRB = 0x00;
  // disable all pull ups
  PORTC = 0x00;
  PORTB = 0x00;
  bus_reading = true;
}

// Set up DDRC and DDRB for writing to the M48T59Y.
void configure_data_bus_for_write() {
  DDRC = 0x0F;
  DDRB = 0x0F;
  bus_reading = false;
}

uint8_t m48t59y_read( uint16_t addr ) {
  if ( !bus_reading )
    configure_data_bus_for_read();

  uint8_t data;
  digitalWrite( M48T59Y_NW, 1 ); // make sure write is de-asserted
  assert_addr( addr );
  NOP();
  digitalWrite( M48T59Y_NE, 0 ); // enable chip
  NOP();
  digitalWrite( M48T59Y_NG, 0 ); // enable output
  NOP();
  data = ((PINB & 0x0F) << 4) | (PINC & 0x0F);
  NOP();
  digitalWrite( M48T59Y_NG, 1 ); // disable output
  digitalWrite( M48T59Y_NE, 1 ); // disable chip
  return data;
}

void m48t59y_write( uint16_t addr, uint8_t data ) {
  if ( bus_reading )
    configure_data_bus_for_write();

  digitalWrite( M48T59Y_NW, 1 ); // make sure write signal is de-asserted
  assert_addr( addr );
  PORTC = (PORTC & 0xF0) | (data & 0xF); // output low nybble of data
  PORTB = (PORTB & 0xF0) | ((data >> 4) & 0xF); // output high nybble of data
  NOP();
  digitalWrite( M48T59Y_NE, 0 ); // enable chip
  NOP();
  digitalWrite( M48T59Y_NW, 0 ); // enable write
  NOP();
  digitalWrite( M48T59Y_NW, 1 ); // disable write
  NOP();
  digitalWrite( M48T59Y_NE, 1 ); // disable chip
}

uint8_t tests_passed, tests_executed;

void report_mismatch( uint16_t addr, uint8_t expected, uint8_t data ) {
  Serial.print("[");
  Serial.print( addr, HEX );
  Serial.print(",e=");
  Serial.print( expected, HEX );
  Serial.print(",a=");
  Serial.print( data, HEX );
  Serial.println("]");
  //delay(250);
}

#define NTESTS 16

static const uint16_t test_addrs[] PROGMEM = {
  0x1bb1, 0x4d9, 0x10b1, 0x936, 0x106d, 0x199f, 0x3b4, 0xd64, 0x160, 0x1db1, 0x1052, 0x1e6e,
  0x61a, 0xabb, 0x75e, 0x1a0e, 
};

static const uint8_t test_data[] PROGMEM = {
0xec, 0x46, 0x67, 0x70, 0x8, 0x52, 0xce, 0xe6, 0x6c, 0x6e, 0x1b, 0xf8,
  0xe2, 0x11, 0xdf, 0x2e, 
};

// Verify that contents of memory are consistent with previous execution
// of simple_rw_tests()
void simple_rw_tests_verify( bool log ) {
  Serial.print( "Verifying data written by simple read/write tests..." );

  uint16_t err_count = 0;

  for ( uint16_t i = 0; i < NTESTS; ++i ) {
    uint16_t addr = pgm_read_word( &test_addrs[i] );
    uint8_t expected = pgm_read_byte( &test_data[i] );
    uint8_t data;

    data = m48t59y_read( addr );
    if ( data != expected ) {
      ++err_count;
      if ( log )
        report_mismatch( addr, expected, data );
    }
  }

  if ( err_count == 0 ) {
    Serial.println( "  Success!" );
    ++tests_passed;
  } else
    Serial.println( "Error(s) accessing memory" );

  ++tests_executed;
}

// Very simple read/write tests
void simple_rw_tests( bool log ) {
  Serial.print( "Simple read/write tests..." );

  uint16_t err_count = 0;

  for ( uint16_t i = 0; i < NTESTS; ++i ) {
    uint16_t addr = pgm_read_word( &test_addrs[i] );
    uint8_t expected = pgm_read_byte( &test_data[i] );
    uint8_t data;

    m48t59y_write( addr, expected );
    data = m48t59y_read( addr );
    if ( data != expected ) {
      ++err_count;
      if ( log )
        report_mismatch( addr, expected, data );
    }

    Serial.print( "." );
  }

  Serial.println();

  if ( err_count == 0 ) {
    Serial.println( "  Success!" );
    ++tests_passed;
  } else
    Serial.println( "Error(s) accessing memory" );

  ++tests_executed;
}

/*
// Verify the contents of memory written previously by writeMem()
void verify_mem( bool log_mismatch = false ) {
  Serial.print( "Verifying memory contents..." );

  uint16_t addr = 0;
  uint16_t mismatch = 0;
  uint8_t data, expected;

  rng_reset();

  while ( addr < M48T59Y_FIRST_REG ) {
    data = m48t59y_read( addr );
    expected = rng_gen();
    if ( data != expected ) {
      ++mismatch;
      if ( log_mismatch )
        report_mismatch( addr, expected, data );
    }
    
    ++addr;
  }

  if ( mismatch > 0 ) {
    Serial.print( mismatch );
    Serial.println( " byte(s) did not match" );
  } else {
    Serial.println( "all bytes matched!" );
    ++tests_passed;
  }

  ++tests_executed;
}

// Write pseudo-random data
void write_mem() {
  Serial.print( "Writing memory contents..." );

  uint16_t addr = 0;
  uint16_t mismatch = 0;
  uint8_t data, expected;

  rng_reset();

  while ( addr < M48T59Y_FIRST_REG ) {
    data = rng_gen();
    m48t59y_write( addr, data );
    ++addr;
  }

  Serial.println( "done" );
}
*/

uint8_t bcd_to_dec( uint8_t bcd ) {
  return (bcd >> 4)*10 + (bcd & 0xF);
}

void set_time_and_date() {
  Serial.println( "Setting time and date..." );

  // set the W bit in the control register
  // (note that we are leaving the calibration bits as 0,
  // and will also clear the stop bit, meaning that the
  // oscillator should run)
  m48t59y_write( M48T59Y_CONTROL, 0x80 );

  // set time and date to 03:40 PM on Jan 3, 2025
  // (which is approximately when I am writing this code)
  m48t59y_write( M48T59Y_SECONDS, 0x00 );
  m48t59y_write( M48T59Y_MINUTES, 0x40 );
  m48t59y_write( M48T59Y_HOUR, 0x15 );
  m48t59y_write( M48T59Y_DAY, 0x06 );
  m48t59y_write( M48T59Y_DATE, 0x03 );
  m48t59y_write( M48T59Y_MONTH, 0x01 );
  m48t59y_write( M48T59Y_YEAR, 0x25 );

  // clear the W bit
  m48t59y_write( M48T59Y_CONTROL, 0x00 );

  // wait one second
  delay( 1000 );
}

// Verify that the clock is running by seeing the second
// counter increase. Given that it was previously set to 0,
// we should need to worry about it rolling over during this test.
void verify_clock_running() {
  Serial.println( "Verifying that clock is counting up..." );

  // set R bit in control register
  m48t59y_write( M48T59Y_CONTROL, 0x40 );

  // read current value of seconds register (will probably be 1)
  uint8_t cur_sec;
  cur_sec = m48t59y_read( M48T59Y_SECONDS );
  cur_sec = bcd_to_dec( cur_sec );
  Serial.print( "  initial seconds=" );
  Serial.print( (uint16_t) cur_sec );
  Serial.println();

  // clear R bit so that registers are updated again
  m48t59y_write( M48T59Y_CONTROL, 0x00 );

  // wait for three seconds
  delay( 3000 );

  // set R bit in control register again
  m48t59y_write( M48T59Y_CONTROL, 0x40 );

  // read updated second count
  uint8_t now_sec = m48t59y_read( M48T59Y_SECONDS );
  now_sec = bcd_to_dec( now_sec );
  Serial.print( "  updated seconds (after 3s delay)=" );
  Serial.print( (uint16_t) now_sec );
  Serial.println();

  // expect the updated second count to have increased by 3 or 4
  if ( cur_sec + 3 == now_sec || cur_sec + 4 == now_sec ) {
    Serial.println( "Passed, seconds seem to be updating" );
    ++tests_passed;
  } else {
    Serial.println( "Failed, seconds are not updating correctly?" );
  }

/*
  // clear R bit again
  configure_data_bus_for_write();
  m48t59y_write( M48T59Y_CONTROL, 0x00 );
  configure_data_bus_for_read();
*/

  ++tests_executed;
}

void run_tests( bool log ) {
  //verify_mem( log );
  simple_rw_tests_verify( log );
  simple_rw_tests( log );
  simple_rw_tests_verify( log );
  //write_mem();
  //verify_mem( log );
  //set_time_and_date();
  //verify_clock_running();

  Serial.print( tests_passed );
  Serial.print( "/" );
  Serial.print( tests_executed );
  Serial.println( " tests passed" );

  configure_data_bus_for_read();
}

void setup() {
  // set shift register and M48T59Y control pins as outputs
  pinMode( SHIFTREG_DATA, OUTPUT );
  pinMode( SHIFTREG_SCLK, OUTPUT );
  pinMode( SHIFTREG_RCLK, OUTPUT );
  pinMode( M48T59Y_NW, OUTPUT );
  pinMode( M48T59Y_NG, OUTPUT );
  pinMode( M48T59Y_NE, OUTPUT );

  // set serial and register clock low initially
  digitalWrite( SHIFTREG_SCLK, 0 );
  digitalWrite( SHIFTREG_RCLK, 0 );

  // de-select M48T59Y, disable output, disable write
  digitalWrite( M48T59Y_NW, 1 );
  digitalWrite( M48T59Y_NG, 1 );
  digitalWrite( M48T59Y_NE, 1 );

  // set data bus pins to input
  configure_data_bus_for_read();

  // use serial output to report results of tests
  Serial.begin( 9600 );
  //Serial.println( "Hello" );

  //runTests();
}

void loop() {
  if ( (PINC & GO) == 0 ) {
    // GO button was pressed
    Serial.println( "GO pressed" );

    // Check whether LOG button is pressed
    bool log = ( (PINC & LOG) == 0 );
    if ( log )
      Serial.println( "LOG pressed, will log data mismatches" );

    tests_passed = 0;
    tests_executed = 0;
    
    run_tests( log );
  }
}
