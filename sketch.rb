#!/usr/bin/env ruby


require 'serialport'

@port = SerialPort.new(
	"/dev/tty.usbserial-DJ002YGH",
	9600,
	8,
	1,
	SerialPort::NONE
)

p @port

#@port.flush_input
#@port.flush_output

@port << "MAX\r"
while l = @port.gets
	fwd, ref, swr = *l.match(/([\d.]+)W ([\d.]+)W (\d+)/).captures.map {|i| i.to_f }
	p [fwd, ref, swr]
	@port << "MAX\r"
end
