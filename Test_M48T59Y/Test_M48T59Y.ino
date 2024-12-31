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
//   Arduino A0..A7 (controlled as AVR PORTC):
//      connected to D0..D7 on the M48T59Y (its data bus pins)

#define SHIFTREG_DATA 2
#define SHIFTREG_SCLK 3
#define SHIFTREG_RCLK 4

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

void setup() {
  pinMode( SHIFTREG_DATA, OUTPUT );
  pinMode( SHIFTREG_SCLK, OUTPUT );
  pinMode( SHIFTREG_RCLK, OUTPUT );

  digitalWrite( SHIFTREG_SCLK, 0 );
  digitalWrite( SHIFTREG_RCLK, 0 );
}

uint16_t test = 0;

void loop() {
  assertAddr( test );
  delay( 100 );
  ++test;
}
