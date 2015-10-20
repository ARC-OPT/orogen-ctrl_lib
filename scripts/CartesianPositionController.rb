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

   controller.field_names                = ["X", "Y", "Z", "rotZ", "rotY", "rotZ"]
   controller.initial_prop_gain          = prop_gain
   controller.initial_dead_zone          = dead_zone
   controller.initial_ff_gain            = ff_gain
   controller.initial_max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint = Types::Base::Samples::RigidBodyState.new
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

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true 
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

      print "Goal Pose:       X: #{'%.04f' % setpoint.position[0]}  Y: #{'%.04f' % setpoint.position[1]}  Z: #{'%.04f' % setpoint.position[2]} "
      print "rotX: #{'%.04f' % setpoint.orientation.to_euler[2]} rotY: #{'%.04f' % setpoint.orientation.to_euler[1]} rotZ: #{'%.04f' % setpoint.orientation.to_euler[0]}\n"
      print "Control Output: vx: #{'%.04f' % control_output.velocity[0]} vy: #{'%.04f' % control_output.velocity[1]} vz: #{'%.04f' % control_output.velocity[2]} "
      print "r_vx: #{'%.04f' % control_output.angular_velocity[0]} r_vy: #{'%.04f' % control_output.angular_velocity[1]} r_vz: #{'%.04f' % control_output.angular_velocity[2]}\n"
      print "Actual Position: X: #{'%.04f' % feedback.position[0]}  Y: #{'%.04f' % feedback.position[1]}  Z: #{'%.04f' % feedback.position[2]} "
      print "rotX: #{'%.04f' % feedback.orientation.to_euler[2]} rotY: #{'%.04f' % feedback.orientation.to_euler[1]} rotZ: #{'%.04f' % feedback.orientation.to_euler[0]}\n\n"
      sleep(cycle_time)
   end
end
