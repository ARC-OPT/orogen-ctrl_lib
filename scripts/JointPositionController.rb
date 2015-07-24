require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::JointPositionController" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   propGain         = Types::Base::VectorXd.new(2)
   deadZone         = Types::Base::VectorXd.new(2)
   maxControlOutput = Types::Base::VectorXd.new(2)

   propGain[0],propGain[1]                 = 1.0,1.0
   deadZone[0],deadZone[1]                 = 0.01,0.03
   maxControlOutput[0],maxControlOutput[1] = 0.3,0.5

   controller.jointNames       = ["Joint_1", "Joint_2"]
   controller.propGain         = propGain
   controller.deadZone         = deadZone
   controller.maxControlOutput = maxControlOutput

   controller.configure
   controller.start

   setpoint          = Types::Base::Commands::Joints.new
   setpoint.names    = controller.jointNames
   cmd1              = Types::Base::JointState.new
   cmd2              = Types::Base::JointState.new
   cmd1.position     = 1.0
   cmd2.position     = 2.0
   setpoint.elements = Array[cmd1, cmd2]

   feedback          = Types::Base::Samples::Joints.new
   feedback.names    = controller.jointNames
   state             = Types::Base::JointState.new
   state.position    = 0.0
   feedback.elements = Array[state, state]

   setpoint_writer      = controller.setpoint.writer
   feedback_writer      = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput        = Types::Base::Commands::Joints.new

   setpoint_writer.write(setpoint)

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end

      for i in (0..1) do feedback.elements[i].position += cycle_time*controlOutput.elements[i].speed end    

      print "Goal Position:   Joint_1: #{'%.04f' % setpoint.elements[0].position}, Joint_2: #{'%.04f' % setpoint.elements[1].position}\n"
      print "Control Output:  Joint_1: #{'%.04f' % controlOutput.elements[0].speed}, Joint_2: #{'%.04f' % controlOutput.elements[1].speed}\n"
      print "Actual Position: Joint_1: #{'%.04f' % feedback.elements[0].position}, Joint_2: #{'%.04f' % feedback.elements[1].position}\n\n"
      sleep(cycle_time)
   end
end
