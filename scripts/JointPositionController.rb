require 'orocos'
require 'readline'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::JointPositionController" => "controller" do
   controller = Orocos::TaskContext.get "controller"

   controller.jointNames = ["Joint_1", "Joint_2"]

   propGain = Types::Base::VectorXd.new(2)
   propGain[0] = 1.0;
   propGain[1] = 1.0;
   controller.propGain = propGain

   deadZone = Types::Base::VectorXd.new(2)
   deadZone[0] = 0.01
   deadZone[1] = 0.03
   controller.deadZone = deadZone
   
   maxControlOutput = Types::Base::VectorXd.new(2)
   maxControlOutput[0] = 0.3
   maxControlOutput[1] = 0.5
   controller.maxControlOutput = maxControlOutput

   controller.configure
   controller.start

   setpoint = Types::Base::Commands::Joints.new
   setpoint.names = controller.jointNames
   cmd = Types::Base::JointState.new
   cmd.position = 1.0
   setpoint.elements << cmd
   cmd.position = 2.0
   setpoint.elements << cmd

   setpoint_writer = controller.setpoint.writer
   setpoint_writer.write(setpoint)

   feedback = Types::Base::Samples::Joints.new
   feedback.names = controller.jointNames
   state = Types::Base::JointState.new
   state.position = 0.0
   feedback.elements << state
   state.position = 0.0
   feedback.elements << state

   feedback_writer = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput = Types::Base::Commands::Joints.new
   cycle_time = 0.01

   puts "Press Ctrl-C to stop ..."
   while true
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
      sleep(cycle_time)
         next
      end
      for i in (0..1)
         feedback.elements[i].position = feedback.elements[i].position + cycle_time*controlOutput.elements[i].speed
      end    
      puts "Goal Position: Joint_1: #{setpoint.elements[0].position},  Joint_2: #{setpoint.elements[1].position}"
      puts "Actual Position: Joint_1: #{feedback.elements[0].position},  Joint_2: #{feedback.elements[1].position}"
      puts
      sleep(cycle_time)
   end
end
