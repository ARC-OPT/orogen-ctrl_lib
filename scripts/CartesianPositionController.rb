require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianPositionController" => "controller" do

   controller = Orocos::TaskContext.get "controller"
   
   propGain         = Types::Base::VectorXd.new(6)
   deadZone         = Types::Base::VectorXd.new(6)
   maxControlOutput = Types::Base::VectorXd.new(6)

   for i in (0..5) do 
      propGain[i]         = 1.0 
      deadZone[i]         = 0.01
      maxControlOutput[i] = 0.5
   end

   controller.propGain         = propGain
   controller.deadZone         = deadZone
   controller.maxControlOutput = maxControlOutput

   controller.configure
   controller.start

   setpoint = Types::Base::Samples::RigidBodyState.new
   setpoint.position[0],setpoint.position[1],setpoint.position[2] = 1.0,2.0,3.0
   setpoint.orientation =  Eigen::Quaternion.from_angle_axis(1.571, Eigen::Vector3.UnitX)

   feedback = Types::Base::Samples::RigidBodyState.new
   for i in 0..2 do feedback.position[i] = 0 end
   feedback.orientation.x,feedback.orientation.y,feedback.orientation.z,feedback.orientation.w = 0,0,0,1
   euler = Types::Base::Vector3d.new(3)
   euler[0],euler[1],euler[2] = 0,0,0

   setpoint_writer      = controller.setpoint.writer
   feedback_writer      = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader 
   controlOutput        = Types::Base::Samples::RigidBodyState.new

   setpoint_writer.write(setpoint)

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true 
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do 
         feedback.position[i] += cycle_time*controlOutput.velocity[i] 
         euler[i] += cycle_time*controlOutput.angular_velocity[i]
      end
      feedback.orientation.from_euler(euler,0,1,2)

      print "Goal Pose:       X: #{'%.04f' % setpoint.position[0]}  Y: #{'%.04f' % setpoint.position[1]}  Z: #{'%.04f' % setpoint.position[2]} "
      print "rotX: #{'%.04f' % setpoint.orientation.to_euler[2]} rotY: #{'%.04f' % setpoint.orientation.to_euler[1]} rotZ: #{'%.04f' % setpoint.orientation.to_euler[0]}\n"
      print "Control Output: vx: #{'%.04f' % controlOutput.velocity[0]} vy: #{'%.04f' % controlOutput.velocity[1]} vz: #{'%.04f' % controlOutput.velocity[2]} "
      print "r_vx: #{'%.04f' % controlOutput.angular_velocity[0]} r_vy: #{'%.04f' % controlOutput.angular_velocity[1]} r_vz: #{'%.04f' % controlOutput.angular_velocity[2]}\n"
      print "Actual Position: X: #{'%.04f' % feedback.position[0]}  Y: #{'%.04f' % feedback.position[1]}  Z: #{'%.04f' % feedback.position[2]} "
      print "rotX: #{'%.04f' % feedback.orientation.to_euler[2]} rotY: #{'%.04f' % feedback.orientation.to_euler[1]} rotZ: #{'%.04f' % feedback.orientation.to_euler[0]}\n\n"
      sleep(cycle_time)
   end
end
