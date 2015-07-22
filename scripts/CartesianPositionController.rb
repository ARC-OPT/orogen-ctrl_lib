require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianPositionController" => "controller" do
   controller = Orocos::TaskContext.get "controller"
   
   propGain = Types::Base::VectorXd.new(6)
   for i in (0..5) do propGain[i] = 1.0 end
   controller.propGain = propGain

   deadZone = Types::Base::VectorXd.new(6)
   for i in (0..5) do deadZone[i] = 0.01 end
   controller.deadZone = deadZone
   
   maxControlOutput = Types::Base::VectorXd.new(6)
   for i in (0..5) do maxControlOutput[i] = 0.5 end
   controller.maxControlOutput = maxControlOutput

   controller.configure
   controller.start

   setpoint = Types::Base::Samples::RigidBodyState.new
   setpoint.position[0] = 1.0
   setpoint.position[1] = 2.0
   setpoint.position[2] = 3.0
   setpoint.orientation =  Eigen::Quaternion.from_angle_axis(1.571, Eigen::Vector3.UnitX)
   setpoint_writer = controller.setpoint.writer
   setpoint_writer.write(setpoint)

   feedback = Types::Base::Samples::RigidBodyState.new
   for i in 0..2 do feedback.position[i] = 0 end
   feedback.orientation.x = 0
   feedback.orientation.y = 0
   feedback.orientation.z = 0
   feedback.orientation.w = 1 
   euler = Types::Base::Vector3d.new(3)
   euler[0] = 0
   euler[1] = 0
   euler[2] = 0

   feedback_writer = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput = Types::Base::Samples::RigidBodyState.new
   cycle_time = 0.01
   sleep(cycle_time)

   puts "Press Ctrl-C to stop ..."
   while true 
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end
      for i in (0..2) do feedback.position[i] = feedback.position[i] + cycle_time*controlOutput.velocity[i] end
      for i in (0..2) do euler[i] = euler[i] + cycle_time*controlOutput.angular_velocity[i] end
      feedback.orientation.from_euler(euler,0,1,2)

      print "Goal Pose: X: #{setpoint.position[0]} Y: #{setpoint.position[1]} Z: #{setpoint.position[2]} "
      print "rotX: #{setpoint.orientation.to_euler[2]} rotY: #{setpoint.orientation.to_euler[1]} rotZ: #{setpoint.orientation.to_euler[0]}\n"
      print "Control Output: vx: #{'%.04f' % controlOutput.velocity[0]} vy: #{'%.04f' % controlOutput.velocity[1]} vz: #{'%.04f' % controlOutput.velocity[2]} "
      print "rot_vx: #{'%.04f' % controlOutput.angular_velocity[0]} rot_vy: #{'%.04f' % controlOutput.angular_velocity[1]} rot_vz: #{'%.04f' % controlOutput.angular_velocity[2]}\n"
      print "Actual Position: X: #{'%.04f' % feedback.position[0]} Y: #{'%.04f' % feedback.position[1]} Z: #{'%.04f' % feedback.position[2]} "
      print "rotX: #{'%.04f' % feedback.orientation.to_euler[2]} rotY: #{'%.04f' % feedback.orientation.to_euler[1]} rotZ: #{'%.04f' % feedback.orientation.to_euler[0]}\n\n"
      sleep(cycle_time)
   end
end
