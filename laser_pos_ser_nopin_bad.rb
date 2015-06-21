#!/usr/bin/env mruby

daemon(true, true)

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
  
  def initialize(config, port = nil)
    if File.exist?(config) then
      @config = YAML.load_file(config)
      raise WSArgError, "Cannot setup hostname" if not @config[:hostname] 
      raise WSArgError, "Cannot setup port" if not @config[:port]
      raise WSArgError, "Cannot setup index" if not @config[:index]
    else
      raise WSArgError, "Cannot find configuration file #{config}"
    end
    puts inspect
  end

  def serve(opt_block = nil)
    @server = TCPServer.new(@config[:hostname], @config[:port])
    if @server
      while(1)
        begin
          session = @server.accept_nonblock
          request = session.gets
          if request.is_a?(String)
            @resource = request.gsub(/GET\ \//, '').gsub(/\ HTTP.*/, '').chomp
            yield(@resource, session)
          end
          close
        rescue
          if opt_block.respond_to?(:call)
            opt_block.call
          end
        end
      end    
    else
      puts @config
      raise WSRunError, "Cannot find @server??"
    end
  ensure
    close
  end

  def send_api(request)
    name, tokens = parse_query(request)
    if methods.include?(name.to_sym)
      ret = self.send(name, *tokens)
      return ret
    else
      return nil
    end
  end

  def close
    puts " = Closing server..."
    if @server
      @server.close
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

  def inspect
    return <<-EOS
=======================================================
Mechatronix Web2Serial Server - a /dev/null creation ;)
=======================================================

 = Configuration
   Webserver  = http://#{@config[:hostname]}:#{@config[:port]}
   Index page = #{@config[:index]}

= Requests ============================================
EOS
  end 
end

if __FILE__ == $0 then
  begin

    cam = RaspiCam.new(config[:frame_width]).open

    ser = SerialPort.new(config[:serialport], config[:baudrate]).open

    ws = MWebSerialServer.new("laser_pos.yml")

    #ENABLE_PIN = 3
    #Raspberry::Core.pin_mode(ENABLE_PIN, Raspberry::INPUT)
    #while Raspberry::Core.digital_read(ENABLE_PIN) == Raspberry::LOW do
    #  sleep(0.1)
    #end
    sleep 0.5
    ser.command('/') {|txt| p txt} 
    
    #### API definitions ###################
    
    ws.api("api/up") do
      ret = ""
      ser.command("200V") { |txt| ret = txt }
      ret
    end
    ws.api("api/down") do
      ret = ""
      ser.command("200v") { |txt| ret = txt }
      ret
    end
    ws.api("api/left") do
      ret = ""
      ser.command("200Y") { |txt| ret = txt }
      ret
    end
    ws.api("api/right") do
      ret = ""
      ser.command("200y") { |txt| ret = txt }
      ret
    end
    ws.api("api/brake") do
      ret = ""
      ser.command("0V0Y") { |txt| ret = txt }
      ret
    end
    ws.api("api/stop") do
      ret = ""
      ser.command("/") { |txt| ret = txt }
      ret
    end
    ws.api("api/laser") do
      ret = ""
      ser.command("A") { |txt| ret = txt }
      ret
    end
    ws.api("api/man") do
      ret = ""
      ser.command("A") { |txt| ret = txt }
      ret
    end
    ws.api("api/query") do 
      ret = ""
      ser.command("?") { |txt| ret = txt }
      ret
    end
    ws.api("api/photo") do
      frame_width = config[:frame_width]
      frame_height  = 3/4 * frame_width

      p1 = [config[:center_x] - config[:dz_x]/2, config[:center_y] + config[:dz_y]/2]
      p2 = [config[:center_x] + config[:dz_x]/2, config[:center_y] - config[:dz_y]/2]

      p1.map! {|e| ((e + 100) * 255.0/200.0).round }
      p2.map! {|e| ((e + 100) * 255.0/200.0).round }

      p1[0] = (p1[0] * frame_width/255.0).round
      p2[0] = (p2[0] * frame_width/255.0).round
      p1[1] = (p1[1] * frame_height/255.0).round
      p2[1] = (p2[1] * frame_height/255.0).round

      p1[1] = frame_height - p1[1]
      p2[1] = frame_height - p2[1]
      cam.set_rect(p1[0], p1[1], p2[0], p2[1])
      cam.save_image config[:save_image] 
      cam.reset_rect
      "updated photo"
    end

    # laser_loop = Proc.new do
    #   ser.command("?") { |txt| 
    #     if txt =~ /state:\s([0-9]+)/
    #       if $1 == "2"
    #         cmd = {}
    #         begin
    #           pos = control(cam.position, config)
    #           cmd[:x] = "#{pos[0].abs}#{pos[0] > 0 ? 'X' : 'x'}"
    #           cmd[:c] = "#{pos[1].abs}#{pos[1] > 0 ? 'c' : 'C'}"
    #         rescue RaspicamError
    #           cmd[:x] = '0x'
    #           cmd[:c] = '0c'
    #         rescue => e
    #           p e
    #         end
    #         cmd_str = cmd[:x] + cmd[:c]
    #         puts "cmd = #{cmd_str}"
    #         # Communicating via Serial
    #         begin
    #           #if Raspberry::Core.read_digital(ENABLE_PIN) == Raspberry::HIGH
    #             ser.command(cmd_str)
    #           #else
    #           #  print '.'
    #           #end
    #         rescue Exception => e
    #           p e
    #         end
    #       end
    #     end
    #   }
    # end
    
    ######################################## 

    ws.serve(laser_loop) { |response, session|
    #ws.serve { |response, session| 
      #puts response
      case response
      when ""
        session.print "HTTP/1.1 200/OK\r\nContent-type:text/html\r\n\r\n"
        session.print File.open(config[:index], "rb").read
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
        session.print "HTTP/1.1 200/OK\r\nContent-type:text/html\r\n\r\n"
        session.print File.open("static/favicon.ico", "rb").read  
        session.close
      else
        if /api/.match(response) then
          reply = ws.send_api(response)
          if reply then
            session.print "HTTP/1.1 200/OK\r\nContent-type:text/html\r\n\r\n"
            #serial_response = ser.command(reply)
            #session.print serial_response
            puts "Reply = #{reply}"
            session.print reply
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
