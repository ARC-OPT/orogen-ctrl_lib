require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")
Orocos.load_typekit("wbc")

Orocos.run do # "ctrl_lib::CartesianForceController" => "controller" do

   controller = Orocos::TaskContext.get "orogen_default_ctrl_lib__CartesianPositionController"

   p_gain            = Types::Base::VectorXd.new(6)
   dead_zone          = Types::Base::VectorXd.new(6)
   max_control_output = Types::Base::VectorXd.new(6)

   for i in (0..5) do
      p_gain[i]             = 1.0
      dead_zone[i]          = 0.01
      max_control_output[i] = 0.5
   end

   controller.field_names        = ["X", "Y", "Z", "rotZ", "rotY", "rotZ"]
   controller.p_gain             = p_gain
   controller.dead_zone          = dead_zone
   controller.max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint = Types::Base::Samples::Wrench.new
   setpoint.time = Types::Base::Time.now
   setpoint.force[0],setpoint.force[1],setpoint.force[2] = 1.0,2.0,3.0
   setpoint.torque[0],setpoint.torque[1],setpoint.torque[2] = 0.0,0.0,0.0

   feedback = Types::Base::Samples::Wrench.new
   feedback.force[0],feedback.force[1],feedback.force[2] = 0.0,0.0,0.0
   feedback.torque[0],feedback.torque[1],feedback.torque[2] = 0.0,0.0,0.0

   setpoint_writer      = controller.setpoint.writer
   feedback_writer      = controller.feedback.writer
   control_output_reader = controller.control_output.reader
   control_output        = Types::Wbc::CartesianState.new

   setpoint_writer.write(setpoint)

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true
      feedback.time = Types.base.Time.now
      feedback_writer.write(feedback)
      if not control_output_reader.read_new(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do
         feedback.force[i] += cycle_time*control_output.twist.linear[i]
         feedback.torque[i] += cycle_time*control_output.twist.angular[i]
      end

      print "Goal Wrench (x/y/z/rx/ry/rz): "
      setpoint.force.data.each do |p| print "#{'%.04f' % p} " end
      setpoint.torque.data.each do |p| print "#{'%.04f' % p} " end

      print "\nMax control output:         "
      max_control_output.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nDead zone:                  "
      dead_zone.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nControl Output:             "
      control_output.twist.linear.data.each do |v| print "#{'%.04f' % v} " end
      control_output.twist.angular.data.each do |v| print "#{'%.04f' % v} " end

      print "\nActual Wrench:            "
      feedback.force.data.each do |p| print "#{'%.04f' % p} " end
      feedback.torque.data.each do |p| print "#{'%.04f' % p} " end
      print "\n\n"
      sleep(cycle_time)
   end
end
