require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianRadialPotentialFields" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   prop_gain             = Types::Base::VectorXd.new(3)
   max_control_output    = Types::Base::VectorXd.new(3)

   for i in (0..2) do
      prop_gain[i]             = 1.0
      max_control_output[i]    = 0.3
   end

   controller.field_names        = ["X", "Y", "Z"]
   controller.prop_gain          = prop_gain
   controller.max_control_output = max_control_output
   controller.influence_distance = 1.0

   controller.configure
   controller.start

   feedback = Types::Base::Samples::RigidBodyState.new
   feedback.position[0],feedback.position[1],feedback.position[2] = 0,0,0.1
   feedback.orientation.x,feedback.orientation.y,feedback.orientation.z,feedback.orientation.w = 0,0,0,1

   pot_field = Types::Base::Samples::RigidBodyState.new
   pot_field.position[0],pot_field.position[1],pot_field.position[2] = 0,0,0

   feedback_writer        = controller.feedback.writer
   pot_field_writer       = controller.pot_field_centers.writer
   control_output_reader  = controller.control_output.reader
   control_output         = Types::Base::Samples::RigidBodyState.new

   pot_field_writer.write([pot_field])

   cycle_time = 0.01
   puts("Press Ctrl-C to stop ...")
   while true
      feedback_writer.write(feedback)
      if not control_output_reader.read(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do feedback.position[i] += cycle_time*control_output.velocity[i] end

      print "Potential Field position:  "
      pot_field.position.data.each do |d| print "#{'%.04f' % d} " end
      print "\nMax control output:        "
      max_control_output.to_a.each do |m| print "#{'%.04f' % m} " end
      print "\nControl Output:            "
      control_output.velocity.data.each do |v| print "#{'%.04f' % v} " end
      print "\nActual Position:           "
      feedback.position.data.each do |p| print "#{'%.04f' % p} " end
      print "\n\n"
      sleep(cycle_time)
   end

end
