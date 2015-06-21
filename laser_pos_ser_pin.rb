#!/usr/bin/env mruby

module YAML
  def YAML.load_file(f)
    YAML.load(File.open(f,"rb").read)
  end
end

config = YAML.load_file("laser_pos.yml")


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

Signal.trap(:INT) do 
  puts "Terminating communication"
  running = false
end

# WebServer

class WSArgError < ArgumentError; end
class WSRunError < RuntimeError; end

class MWebSerialServer
  
  def initialize(config, serial = nil)
    if File.exist?(config) then
      @config = YAML.load_file(config)
      raise WSArgError, "Cannot setup hostname" if not @config[:hostname] 
      raise WSArgError, "Cannot setup port" if not @config[:port]
      raise WSArgError, "Cannot setup index" if not @config[:index]
      raise WSArgError, "Cannot setup index" if not @config[:serial]
      raise WSArgError, "Cannot setup index" if not @config[:baud]
    else
      raise WSArgError, "Cannot find configuration file #{config}"
    end
    if serial
      @serial = serial
    end
    puts inspect
  end

  def serve(opt_block = nil)
    @server = TCPServer.new(@config[:hostname], @config[:port])
    #puts " = Created web server"
    @serial = SerialPort.new(@config[:serial], @config[:baud]).open unless @serial
    @serial.flush
    #puts " = Created serial server"
    
    @serial.command("?") { |r| puts YAML.load(r).inspect }

    if @server
      while (session = @server.accept)
        request = session.gets
        if request.is_a?(String)
          @resource = request.gsub(/GET\ \//, '').gsub(/\ HTTP.*/, '').chomp
          yield(@resource, session)
        end

        if opt_block and opt_block.respond_to?(:call)
          opt_block.call
        end
      end
      close
    else
      puts @config
      raise WSRunError, "Cannot find @server??"
    end
  ensure
    close
  end

  def send_api(request)
    name, tokens = parse_query(request)
    # print "Sending: #{request} -> #{name}(#{tokens})"
    if methods.include?(name.to_sym)
      # puts "OK"
      ret = self.send(name, *tokens)
      # puts ret
      return ret
    else
      #puts "???"
      return nil
    end
  end

  def close
    puts " = Closing server..."
    if @server
      @server.close
    end
    puts " = Closing serial..."
    if @serial
      @serial.close
    end
    puts "BYE!"
  end

  def parse_query(s)
    tokens = s.split("/")
    return tokens[0..1].join("_"), tokens[2..-1]
  end

  def api(s, &block)
    name, tokens = parse_query(s)
    MWebSerialServer.define_method(name) { |*tokens| yield(tokens) }
    puts " = Define new API -> #{name}(#{tokens.flatten})" if methods.include?(name.to_sym)
  end

  def command(c)
    if @serial
      result = @serial.command(c) { |r| r }
      @serial.flush
      return result
    else
      #print " [redirected] " 
      return c
    end
  end

  def inspect
    return <<-EOS
=======================================================
Mechatronix Web2Serial Server - a /dev/null creation ;)
=======================================================

 = Configuration
   Webserver  = http://#{@config[:hostname]}:#{@config[:port]}
   Index page = #{@config[:index]}
   Serial     = #{@config[:serial]} @ #{@config[:baud]}

= Requests ============================================
EOS
  end 
end

if __FILE__ == $0 then
  begin

    cam = RaspiCam.new(config[:frame_width]).open

    ser = SerialPort.new(config[:serialport], config[:baudrate]).open

    ws = MWebSerialServer.new("./config.yaml", ser)

    ENABLE_PIN = 3
    Raspberry::Core.pin_mode(ENABLE_PIN, Raspberry::INPUT)
    while Raspberry::Core.digital_read(ENABLE_PIN) == Raspberry::LOW do
      sleep(0.1)
    end
    ser.command('/') {|txt| p txt} 
    
    #### API definitions ###################
    
    ws.api("api/up") do
      "200X"
    end
    ws.api("api/down") do
      "200x"
    end
    ws.api("api/left") do
      "200C"
    end
    ws.api("api/right") do
      "200c"
    end
    ws.api("api/brake") do
      "0C0X"
    end
    ws.api("api/stop") do
      "/"
    end
    ws.api("api/laser") do
      "A"
    end
    ws.api("api/man") do
      "0X0C"
    end
    ws.api("api/query") do 
      "?"
    end
    ws.api("api/query") do
      cam.save_image(config[:save_image])
    end

    laser_loop = Proc.new do
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
      puts "cmd = #{cmd_str}"
      # Communicating via Serial
      begin
        if Raspberry::Core.read_digital(ENABLE_PIN) == Raspberry::HIGH
          ser.command(cmd_str)
        else
          print '.'
        end
      rescue Exception => e
        p e
      end
    end
    
    ######################################## 
    #puts ws.methods.sort
    #puts ws.methods.include?(:api_move)

    ws.serve(laser_loop) { |response, session| 
      #puts response
      case response
      when ""
        session.print "HTTP/1.1 200/OK\r\nContent-type:text/html\r\n\r\n"
        #File.foreach("index.html") { |line| session.print line }
        session.print File.open("index.html", "rb").read
        session.close
      when /^static\//
        if File.exist?(response)
          session.print "HTTP/1.1 200/OK\r\nContent-type:text/html\r\n\r\n"
          session.print File.open(response, "rb").read  
          session.close
        else
          session.print "HTTP/1.1 404/Object Not Found\r\nMWebServer /dev/null\r\n\r\n"
          session.print "404 - Resource cannot be found."
          session.close
        end
      when "favicon.ico"
        session.print "HTTP/1.1 404/Object Not Found\r\nMWebServer /dev/null\r\n\r\n"
        session.close
      else
        if /api/.match(response) then
          reply = ws.send_api(response)
          if reply then
            ## Execute serial command
            serial_response  = ws.command(reply)
            ## 
            session.print "HTTP/1.1 200/OK\r\nContent-type:text/html\r\n\r\n"
            puts " *** "
            puts reply + " = "
            puts serial_response
            puts " *** "
            session.print serial_response
            session.close
          else
            session.print "HTTP/1.1 404/Object Not Found\r\nMWebServer /dev/null\r\n\r\n"
            session.close
          end
        else 
          session.print "HTTP/1.1 404/Object Not Found\r\nMWebServer /dev/null\r\n\r\n"
          session.print "404 - Resource cannot be found."
          session.close
        end
      end
    }
  end
end
