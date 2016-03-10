require 'orocos'
require 'readline'

Orocos.initialize
Orocos.load_typekit("base")


Orocos.run "ctrl_lib::CartesianForceController" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   prop_gain          = Types::Base::VectorXd.new(6)
   dead_zone          = Types::Base::VectorXd.new(6)
   max_control_output = Types::Base::VectorXd.new(6)

   for i in (0..5) do
      prop_gain[i]          = 1.0
      dead_zone[i]          = 0.01
      max_control_output[i] = 0.5
   end

   controller.field_names        = ["X", "Y", "Z", "rotZ", "rotY", "rotZ"]
   controller.prop_gain          = prop_gain
   controller.dead_zone          = dead_zone
   controller.max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint_writer      = controller.setpoint.writer
   feedback_writer      = controller.feedback.writer
   control_output_reader = controller.control_output.reader

   setpoint = Types::Base::Samples::Wrench.new
   feedback = Types::Base::Samples::Wrench.new
   control_output        = Types::Base::Samples::RigidBodyState.new
 
   setpoint.time = Types::Base::Time.now
   setpoint.force = Types::Base::Vector3d.new(2,3,4)
   setpoint.torque = Types::Base::Vector3d.new(0,0,0)
   feedback.force = Types::Base::Vector3d.new(0,0,0)
   feedback.torque = Types::Base::Vector3d.new(0,0,0)

   setpoint_writer.write(setpoint)

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true
      feedback.time = Types::Base::Time.now
      feedback_writer.write(feedback)
      if not control_output_reader.read(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do
         feedback.force[i] += cycle_time*control_output.velocity[i]
         feedback.torque[i] += cycle_time*control_output.angular_velocity[i]
      end

      print "Goal Force (Fx/Fy/Fz/Tx/Ty/Tz): "
      setpoint.force.data.each do |p| print "#{'%.04f' % p} " end
      setpoint.torque.data.each do |e| print "#{'%.04f' % e} " end

      print "\nMax control output:         "
      max_control_output.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nDead zone:                  "
      dead_zone.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nControl Output:             "
      control_output.velocity.data.each do |v| print "#{'%.04f' % v} " end
      control_output.angular_velocity.data.each do |v| print "#{'%.04f' % v} " end

      print "\nActual Force:            "
      feedback.force.data.each do |p| print "#{'%.04f' % p} " end
      feedback.torque.data.each do |e| print "#{'%.04f' % e} " end
      print "\n\n"

     sleep cycle_time
   end
end
