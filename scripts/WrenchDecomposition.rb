require 'orocos'
require 'readline'

Orocos.initialize

def createWrench(fx, fy, fz, tx, ty, tz)
   w = Types.base.Wrench.new
   w.force = Types.base.Vector3d.new(fx,fy,fz)
   w.torque = Types.base.Vector3d.new(tx,ty,tz)
   return w
end

Orocos.run "ctrl_lib::WrenchDecomposition" => "wrench_decomp" do

   wrench_decomp = Orocos::TaskContext.get "wrench_decomp"
   wrench_decomp.wrench_names = ["ft_sensor_right", "ft_sensor_left"]
   wrench_decomp.configure
   wrench_decomp.start
  
   Readline.readline("Press Enter to send a wrench vector")
 
   wrenches = Types.base.samples.Wrenches.new
   wrenches.elements << createWrench(1,2,3,4,5,6)
   wrenches.names << "ft_sensor_right"
   wrenches.elements << createWrench(7,8,9,10,11,12)
   wrenches.names << "ft_sensor_left"

   writer = wrench_decomp.wrenches.writer

   (0..20).each do |i|
      wrenches.time = Types.base.Time.now
      writer.write wrenches
      sleep 0.5
   end
  
   Readline.readline("Press Enter to reconfigure")

   wrench_decomp.stop
   wrench_decomp.cleanup
   wrench_decomp.configure
   wrench_decomp.start
   

   (0..20).each do |i|
      wrenches.time = Types.base.Time.now
      writer.write wrenches
      sleep 0.5
   end
  
   Readline.readline("Press Enter to add a wrench and reconfigure")

   wrench_decomp.stop
   wrench_decomp.cleanup
   wrench_decomp.wrench_names = ["ft_sensor_right", "ft_sensor_left", "ft_sensor_middle"]
   wrench_decomp.configure
   wrench_decomp.start

   wrenches.elements << createWrench(13,14,15,16,17,18)
   wrenches.names << "ft_sensor_middle"

   (0..20).each do |i|
      wrenches.time = Types.base.Time.now
      writer.write wrenches
      sleep 0.5
   end
  
   Readline.readline("Press Enter to test not configured wrench names")

   wrench_decomp.stop
   wrench_decomp.cleanup
   wrench_decomp.wrench_names = ["ft_sensor_right", "ft_sensor_left"]
   wrench_decomp.configure
   wrench_decomp.start

   (0..20).each do |i|
      wrenches.time = Types.base.Time.now
      writer.write wrenches
      sleep 0.5
   end
  
   Readline.readline("Press Enter to exit")
end
