require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::JointRadialPotentialFields" => "controller" do
   
   controller = Orocos::TaskContext.get "controller"

   prop_gain              = Types::Base::VectorXd.new(2)
   max_control_output     = Types::Base::VectorXd.new(2)
   max_influence_distance = Types::Base::VectorXd.new(2)
   pot_field_centers      = Types::Base::Commands::Joints.new
   pot_field_centers.names = ["Joint_1", "Joint_2"]

   for i in (0..1) do 
      prop_gain[i]             = 0.1
      max_control_output[i]     = 0.5
      max_influence_distance[i] = 1.0
      state = Types::Base::JointState.new
      state.position = 0.0
      pot_field_centers.elements << state
   end

   controller.field_names                = ["Joint_1", "Joint_2"]
   controller.initial_prop_gain          = prop_gain
   controller.initial_max_control_output = max_control_output
   controller.initial_influence_distance = max_influence_distance
   controller.order                      = 1
   controller.initial_pot_field_centers  = pot_field_centers

   controller.configure
   controller.start

   position          = Types::Base::Samples::Joints.new
   position.names    = controller.field_names
   state                    = Types::Base::JointState.new
   state.position           = 0.01
   position.elements = Array[state, state]

   position_writer = controller.position.writer
   control_output_reader  = controller.control_output.reader
   control_output         = Types::Base::Commands::Joints.new

   cycle_time = 0.01
   puts "Press Ctrl-C to exit..."
   while true
      position_writer.write(position)
      if not control_output_reader.read(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..1) do position.elements[i].position += cycle_time*control_output.elements[i].speed end

      print "Potential Field Centers: Joint_1: #{'%.04f' % pot_field_centers.elements[0].position}, Joint_2: #{'%.04f' % pot_field_centers.elements[1].position}\n"
      print "Control Output:          Joint_1: #{'%.04f' % control_output.elements[0].speed}, Joint_2: #{'%.04f' % control_output.elements[1].speed}\n"
      print "Actual Position:         Joint_1: #{'%.04f' % position.elements[0].position}, Joint_2: #{'%.04f' % position.elements[1].position}\n\n"
      sleep(cycle_time)
   end

end
