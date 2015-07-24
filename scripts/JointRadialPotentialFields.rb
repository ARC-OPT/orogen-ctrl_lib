require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::JointRadialPotentialFields" => "controller" do
   
   controller = Orocos::TaskContext.get "controller"

   propGain             = Types::Base::VectorXd.new(2)
   deadZone             = Types::Base::VectorXd.new(2)
   maxControlOutput     = Types::Base::VectorXd.new(2)
   maxInfluenceDistance = Types::Base::VectorXd.new(2)
   potFieldCenters      = Types::Base::VectorXd.new(2)

   for i in (0..1) do 
      propGain[i]             = 0.1 
      deadZone[i]             = 0.0
      maxControlOutput[i]     = 0.5
      maxInfluenceDistance[i] = 1.0
      potFieldCenters[i]      = 0.0
   end

   controller.jointNames           = ["Joint_1", "Joint_2"]
   controller.propGain             = propGain
   controller.deadZone             = deadZone
   controller.maxControlOutput     = maxControlOutput
   controller.maxInfluenceDistance = maxInfluenceDistance
   controller.order                = 1
   controller.potFieldCenters      = potFieldCenters

   controller.configure
   controller.start

   feedback       = Types::Base::Samples::Joints.new
   feedback.names = controller.jointNames
   state          = Types::Base::JointState.new
   state.position = 0.01
   feedback.elements = Array[state, state]

   feedback_writer      = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput        = Types::Base::Commands::Joints.new

   cycle_time = 0.01
   puts "Press Ctrl-C to exit..."
   while true
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end

      for i in (0..1) do feedback.elements[i].position += cycle_time*controlOutput.elements[i].speed end    

      print "Potential Field Centers: Joint_1: #{'%.04f' % potFieldCenters[0]}, Joint_2: #{'%.04f' % potFieldCenters[1]}\n"
      print "Control Output:          Joint_1: #{'%.04f' % controlOutput.elements[0].speed}, Joint_2: #{'%.04f' % controlOutput.elements[1].speed}\n"
      print "Actual Position:         Joint_1: #{'%.04f' % feedback.elements[0].position}, Joint_2: #{'%.04f' % feedback.elements[1].position}\n\n"
      sleep(cycle_time)
   end

end
