require 'orocos'
require 'pry'
require 'readline'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianRadialPotentialFields" => "controller" do
   controller = Orocos::TaskContext.get "controller"
   
   propGain = Types::Base::VectorXd.new(3)
   for i in (0..2) do propGain[i] = 0.01 end
   controller.propGain = propGain

   deadZone = Types::Base::VectorXd.new(3)
   for i in (0..2) do deadZone[i] = 0.01 end
   controller.deadZone = deadZone
   
   maxControlOutput = Types::Base::VectorXd.new(3)
   for i in (0..2) do maxControlOutput[i] = 0.5 end
   controller.maxControlOutput = maxControlOutput

   maxInfluenceDistance = Types::Base::VectorXd.new(2)
   for i in (0..1) do maxInfluenceDistance[i] = 1.0 end
   controller.maxInfluenceDistance = maxInfluenceDistance
  
   controller.order = 0

   center1 = Types::Base::VectorXd.new(3)
   center2 = Types::Base::VectorXd.new(3)
   potFieldCenters = []
   for i in (0..2) do center1[i] = 0.0 end
   potFieldCenters << center1
   for i in (0..2) do center2[i] = 1.0 end
   center2[2] = 0
   center2[1] = 0
   potFieldCenters << center2
   controller.potFieldCenters = potFieldCenters

   controller.configure
   controller.start
    
   feedback = Types::Base::Samples::RigidBodyState.new
   feedback.position[0] = 0
   feedback.position[1] = 0
   feedback.position[2] = 0.5
   feedback.orientation.x = 0
   feedback.orientation.y = 0
   feedback.orientation.z = 0
   feedback.orientation.w = 1 

   feedback_writer = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput = Types::Base::Samples::RigidBodyState.new
   cycle_time = 0.01
   sleep(cycle_time)

   puts("Press Ctrl-C to stop ...")
   
   while true 
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end
      for i in (0..2) do feedback.position[i] = feedback.position[i] + cycle_time*controlOutput.velocity[i] end
      print "Potential Field 1 position: "
      puts potFieldCenters[0]
      print "Potential Field 2 position: "
      puts potFieldCenters[1]
      print "Control Output: "
      for i in 0..2 do print "#{'%.04f' % controlOutput.velocity[i]} " end     
      print "\nActual Position: "
      for i in 0..2 do print "#{'%.04f' % feedback.position[i]} " end
      print "#{'%.04f' % feedback.orientation.to_euler[0]} #{'%.04f' % feedback.orientation.to_euler[1]} #{'%.04f' % feedback.orientation.to_euler[2]}\n\n"
      sleep(cycle_time)
   end
   
end
