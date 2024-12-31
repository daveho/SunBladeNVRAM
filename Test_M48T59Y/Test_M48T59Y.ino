// Test program for M48T59Y

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
//   Arduino A0..A7 (controlled as AVR PORTC):
//      connected to D0..D7 on the M48T59Y (its data bus pins)

#define SHIFTREG_DATA 2
#define SHIFTREG_SCLK 3
#define SHIFTREG_RCLK 4

#define M48T59Y_NW 5
#define M48T59Y_NG 6
#define M48T59Y_NE 7

void assertAddr(uint16_t addr) {
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

void resetRNG() {
  state = SEED;
  rngNumAvail = 0;
}

uint8_t genRNG() {
  if ( rngNumAvail == 0 ) {
    rngData = mulberry32();
    rngNumAvail = 4;
  }
  uint8_t result = rngData & 0xFF;
  rngData >>= 8;
  --rngNumAvail;
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
  DDRC = 0x00;
}

uint16_t test = 0;

void loop() {
  assertAddr( test );
  delay( 100 );
  ++test;
}
