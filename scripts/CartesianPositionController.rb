require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianPositionController" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   prop_gain          = Types::Base::VectorXd.new(6)
   dead_zone          = Types::Base::VectorXd.new(6)
   max_control_output = Types::Base::VectorXd.new(6)
   ff_gain            = Types::Base::VectorXd.new(6)

   for i in (0..5) do
      prop_gain[i]          = 1.0
      dead_zone[i]          = 0.01
      max_control_output[i] = 0.5
      ff_gain[i]            = 0
   end

   controller.field_names        = ["X", "Y", "Z", "rotZ", "rotY", "rotZ"]
   controller.prop_gain          = prop_gain
   controller.dead_zone          = dead_zone
   controller.ff_gain            = ff_gain
   controller.max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint = Types::Base::Samples::RigidBodyState.new
   setpoint.time = Types::Base::Time.now
   setpoint.position[0],setpoint.position[1],setpoint.position[2] = 1.0,2.0,3.0
   setpoint.orientation =  Eigen::Quaternion.from_angle_axis(1.571, Eigen::Vector3.UnitZ)

   feedback = Types::Base::Samples::RigidBodyState.new
   for i in 0..2 do feedback.position[i] = 0 end
   feedback.orientation.x,feedback.orientation.y,feedback.orientation.z,feedback.orientation.w = 0,0,0,1
   euler = Types::Base::Vector3d.new(3)
   euler[0],euler[1],euler[2] = 0,0,0

   setpoint_writer      = controller.setpoint.writer
   feedback_writer      = controller.feedback.writer
   control_output_reader = controller.control_output.reader
   control_output        = Types::Base::Samples::RigidBodyState.new

   setpoint_writer.write(setpoint)

   controller.startEvaluation(true)

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
         feedback.position[i] += cycle_time*control_output.velocity[i]
         euler[i] += cycle_time*control_output.angular_velocity[i]
      end
      feedback.orientation.from_euler(euler,0,1,2)

      print "Goal Pose (x/y/z/rx/ry/rz): "
      setpoint.position.data.each do |p| print "#{'%.04f' % p} " end
      setpoint.orientation.to_euler.data.each do |e| print "#{'%.04f' % e} " end

      print "\nMax control output:         "
      max_control_output.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nDead zone:                  "
      dead_zone.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nControl Output:             "
      control_output.velocity.data.each do |v| print "#{'%.04f' % v} " end
      control_output.angular_velocity.data.each do |v| print "#{'%.04f' % v} " end

      print "\nActual Position:            "
      feedback.position.data.each do |p| print "#{'%.04f' % p} " end
      feedback.orientation.to_euler.data.each do |e| print "#{'%.04f' % e} " end
      print "\n\n"
      sleep(cycle_time)
   end
end
