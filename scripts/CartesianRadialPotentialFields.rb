require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianRadialPotentialFields" => "controller" do

   controller = Orocos::TaskContext.get "controller"
   
   propGain             = Types::Base::VectorXd.new(3)
   deadZone             = Types::Base::VectorXd.new(3)
   maxControlOutput     = Types::Base::VectorXd.new(3)
   maxInfluenceDistance = Types::Base::VectorXd.new(2)
   center1              = Types::Base::VectorXd.new(3)
   center2              = Types::Base::VectorXd.new(3)
   center1[0],center1[1],center1[2] = 0,0,0
   center2[0],center2[1],center2[2] = 1,0,0

   for i in (0..2) do 
      propGain[i]             = 0.01 
      deadZone[i]             = 0.01
      maxControlOutput[i]     = 0.5 
      maxInfluenceDistance[i] = 1.0
   end

   controller.propGain             = propGain
   controller.deadZone             = deadZone
   controller.maxControlOutput     = maxControlOutput
   controller.maxInfluenceDistance = maxInfluenceDistance
   controller.order                = 0
   controller.potFieldCenters      = Array[center1, center2]

   controller.configure
   controller.start
    
   feedback = Types::Base::Samples::RigidBodyState.new
   feedback.position[0],feedback.position[1],feedback.position[2] = 0,0,0.5
   feedback.orientation.x,feedback.orientation.y,feedback.orientation.z,feedback.orientation.w = 0,0,0,1

   feedback_writer      = controller.feedback.writer
   controlOutput_reader = controller.controlOutput.reader
   controlOutput        = Types::Base::Samples::RigidBodyState.new 

   cycle_time = 0.01
   puts("Press Ctrl-C to stop ...")
   while true 
      feedback_writer.write(feedback)
      if not controlOutput_reader.read(controlOutput)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do feedback.position[i] += cycle_time*controlOutput.velocity[i] end

      print "Potential Field 1 position: #{'%.04f' % controller.potFieldCenters[0][0]} #{'%.04f' % controller.potFieldCenters[0][1]} #{'%.04f' % controller.potFieldCenters[0][2]}\n"
      print "Potential Field 2 position: #{'%.04f' % controller.potFieldCenters[1][0]} #{'%.04f' % controller.potFieldCenters[1][1]} #{'%.04f' % controller.potFieldCenters[1][2]}\n"
      print "Control Output:             #{'%.04f' % controlOutput.velocity[0]} #{'%.04f' % controlOutput.velocity[1]} #{'%.04f' % controlOutput.velocity[2]}\n"
      print "Actual Position:            #{'%.04f' % feedback.position[0]} #{'%.04f' % feedback.position[1]} #{'%.04f' % feedback.position[2]}"
      print " #{'%.04f' % feedback.orientation.to_euler[0]} #{'%.04f' % feedback.orientation.to_euler[1]} #{'%.04f' % feedback.orientation.to_euler[2]}\n\n"
      sleep(cycle_time)
   end
   
end
