require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianRadialPotentialFields" => "controller" do

   controller = Orocos::TaskContext.get "controller"
   
   prop_gain              = Types::Base::VectorXd.new(3)
   max_control_output     = Types::Base::VectorXd.new(3)
   influence_distance     = 1.0
   center1                = Types::Base::Samples::RigidBodyState.new
   center2                = Types::Base::Samples::RigidBodyState.new

   center1.position[0],center1.position[1],center1.position[2] = 0,0,0
   center2.position[0],center2.position[1],center2.position[2] = 1,0,0

   for i in (0..2) do 
      prop_gain[i]             = 1.0
      max_control_output[i]    = 0.3
   end

   controller.field_names                = ["X", "Y", "Z"]
   controller.initial_prop_gain          = prop_gain
   controller.initial_max_control_output = max_control_output
   controller.initial_influence_distance =  influence_distance
   controller.order                      = 1
   controller.initial_pot_field_centers  = Array[center1, center2]

   controller.configure
   controller.start
    
   actual_position = Types::Base::Samples::RigidBodyState.new
   actual_position.position[0],actual_position.position[1],actual_position.position[2] = 0,0,0.5
   actual_position.orientation.x,actual_position.orientation.y,actual_position.orientation.z,actual_position.orientation.w = 0,0,0,1

   actual_position_writer = controller.actual_position.writer
   control_output_reader  = controller.control_output.reader
   control_output         = Types::Base::Samples::RigidBodyState.new

   cycle_time = 0.01
   puts("Press Ctrl-C to stop ...")
   while true 
      actual_position_writer.write(actual_position)
      if not control_output_reader.read(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do actual_position.position[i] += cycle_time*control_output.velocity[i] end

      print "Potential Field 1 position: #{'%.04f' % controller.initial_pot_field_centers[0].position[0]} #{'%.04f' % controller.initial_pot_field_centers[0].position[1]} #{'%.04f' % controller.initial_pot_field_centers[0].position[2]}\n"
      print "Potential Field 2 position: #{'%.04f' % controller.initial_pot_field_centers[1].position[0]} #{'%.04f' % controller.initial_pot_field_centers[1].position[1]} #{'%.04f' % controller.initial_pot_field_centers[1].position[2]}\n"
      print "Control Output:             #{'%.04f' % control_output.velocity[0]} #{'%.04f' % control_output.velocity[1]} #{'%.04f' % control_output.velocity[2]}\n"
      print "Actual Position:            #{'%.04f' % actual_position.position[0]} #{'%.04f' % actual_position.position[1]} #{'%.04f' % actual_position.position[2]}"
      print " #{'%.04f' % actual_position.orientation.to_euler[0]} #{'%.04f' % actual_position.orientation.to_euler[1]} #{'%.04f' % actual_position.orientation.to_euler[2]}\n\n"
      sleep(cycle_time)
   end
   
end
