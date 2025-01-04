#! /usr/bin/env ruby

NTESTS = 16
ROWLEN = 12

puts "#define NTESTS #{NTESTS}"
puts

addrs = {}

print "static const uint16_t test_addrs[] PROGMEM = {\n  "

(0..NTESTS-1).each do |i|
  ok = false
  while !ok
    addr = rand(0x1FF0)
    ok = !addrs.has_key?( addr )
  end
  addrs[addr] = 1

  printf( "0x%x,", addr )
  if i % ROWLEN == ROWLEN-1
    print "\n  "
  else
   print " "
  end
end

puts "\n};"

puts ""

puts "static const uint8_t test_data[] PROGMEM = {"

(0..NTESTS-1).each do |i|
  data = rand(256)
  printf( "0x%x,", data );
  if i % ROWLEN == ROWLEN-1
    print "\n  "
  else
   print " "
  end
end

puts "\n};"
