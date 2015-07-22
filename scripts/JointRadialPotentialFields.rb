require 'orocos'
require 'readline'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::JointRadialPotentialFields" => "controller" do
   
   controller = Orocos::TaskContext.get "controller"
   controller.jointNames = ["Joint_1", "Joint_2"]
   propGain = Types::Base::VectorXd.new(2)
   for i in (0..1) do propGain[i] = 0.1 end
   controller.propGain = propGain

   deadZone = Types::Base::VectorXd.new(2)
   for i in (0..1) do deadZone[i] = 0.01 end
   controller.deadZone = deadZone
   
   maxControlOutput = Types::Base::VectorXd.new(2)
   for i in (0..1) do maxControlOutput[i] = 0.5 end
   controller.maxControlOutput = maxControlOutput

   maxInfluenceDistance = Types::Base::VectorXd.new(2)
   for i in (0..1) do maxInfluenceDistance[i] = 1.0 end
   controller.maxInfluenceDistance = maxInfluenceDistance
  
   controller.order = 1

   potFieldCenters = Types::Base::VectorXd.new(2)
   for i in (0..1) do potFieldCenters[i] = 0.0 end
   controller.potFieldCenters = potFieldCenters

   controller.configure
   controller.start

   cycle_time = 0.01

   feedback = Types::Base::Samples::Joints.new
   feedback.names = controller.jointNames
   state = Types::Base::JointState.new
   state.position = 0.01
   feedback.elements << state
   state.position = 0.01
   feedback.elements << state

   feedback_writer = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput = Types::Base::Commands::Joints.new

   puts "Press Ctrl-C to exit..."
   while true
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end
      for i in (0..1) do feedback.elements[i].position = feedback.elements[i].position + cycle_time*controlOutput.elements[i].speed end    
      puts "Potential Field Centers: Joint_1: #{'%.04f' % potFieldCenters[0]},  Joint_2: #{'%.04f' % potFieldCenters[1]}"
      puts "Control Output: Joint_1: #{'%.04f' % controlOutput.elements[0].speed},  Joint_2: #{'%.04f' % controlOutput.elements[1].speed}"
      puts "Actual Position: Joint_1: #{'%.04f' % feedback.elements[0].position},  Joint_2: #{'%.04f' % feedback.elements[1].position}"
      puts
      sleep(cycle_time)
   end

end
