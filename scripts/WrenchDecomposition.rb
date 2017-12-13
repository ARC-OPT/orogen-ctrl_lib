require 'orocos'
require 'readline'

Orocos.initialize

def createWrench(fx, fy, fz, tx, ty, tz)
   w = Types.base.Wrench.new
   w.force = Types.base.Vector3d.new(fx,fy,fz)
   w.torque = Types.base.Vector3d.new(tx,ty,tz)
   return w
end

def sendWrenches(writer, wrenches, n_loops, increment)
    (0..n_loops).each do |i|
       wrenches.time = Types.base.Time.now
       writer.write wrenches
       sleep 0.5
       wrenches.elements.each do |e|
           (0..2).each{|i| e.force[i] += increment}
           (0..2).each{|i| e.torque[i] += increment}
       end
    end
end

def reconfigure(task, wrench_names)
    task.stop
    task.cleanup
    task.wrench_names = wrench_names
    task.configure
    task.start
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
   sendWrenches(writer, wrenches, 10, 0.01)
   Readline.readline("Press Enter to reconfigure")

   reconfigure(wrench_decomp, ["ft_sensor_right", "ft_sensor_left"])
   sendWrenches(writer, wrenches, 10, 0.01)
   Readline.readline("Press Enter to add a wrench and reconfigure")

   reconfigure(wrench_decomp, ["ft_sensor_right", "ft_sensor_left", "ft_sensor_middle"])
   wrenches.elements << createWrench(13,14,15,16,17,18)
   wrenches.names << "ft_sensor_middle"
   sendWrenches(writer, wrenches, 10, 0.01)
   Readline.readline("Press Enter to test not configured wrench names")

   reconfigure(wrench_decomp, ["ft_sensor_right", "ft_sensor_left"])
   sendWrenches(writer, wrenches, 10, 0.01)
   Readline.readline("Press Enter to exit")
end
