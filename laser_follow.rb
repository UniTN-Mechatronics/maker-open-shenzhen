#!/usr/bin/env mruby

daemon(true, true)

config = YAML.load(File.open("laser_pos.yml", "rb").read)

# Reset 0 axes for position output
# rescaling from -100 to + 100
def rescale(value)
  return (value * 200.0/255.0 - 100.0).round(0)
end

# Calculate output on dedzone basis
# center => center point for 
def deadzone(value, center, dz)
  value = rescale(value) - center
  return ( value.abs < dz/2 ? 0 : (value > 0 ? 1 : -1) * (value.abs - dz/2))
end

def control(ary, config)
  ret = [deadzone(ary[0], config[:center_x], config[:dz_x]), deadzone(ary[1], config[:center_y], config[:dz_y])]
  ret.map! { |e| (e.abs > 100 ? (e > 0 ? 1 : -1) * 100 : e ).to_i }
  return ret.reverse
end

cam = RaspiCam.new(config[:frame_width]).open
ser = SerialPort.new(config[:serialport], config[:baudrate]).open
running = true
begin
  ser.command('/') {|txt| p txt}
rescue TimeoutError => e
  puts e.inspect
end

Signal.trap(:INT) do 
  puts "Terminating communication"
  running = false
end

puts "Begin serial communication with #{config[:serialport]}@#{config[:baudrate]}"
cmd = {}
pos = []
while(running) do 
  # Getting measurement
  begin
    pos = control(cam.position, config)
    cmd[:x] = "#{pos[0].abs}#{pos[0] > 0 ? 'X' : 'x'}"
    cmd[:c] = "#{pos[1].abs}#{pos[1] > 0 ? 'c' : 'C'}"
  rescue RaspicamError
    cmd[:x] = '0x'
    cmd[:c] = '0c'
  rescue => e
    p e
  end
  cmd_str = cmd[:x] + cmd[:c]
  #puts "cmd = #{cmd_str}"
  # Communicating via I2C
  begin
    ser.command(cmd_str) { |p| puts "#{p}" }
  rescue I2CError
    print '.'
  rescue Exception => e
    p e
  end
end

ser.close

