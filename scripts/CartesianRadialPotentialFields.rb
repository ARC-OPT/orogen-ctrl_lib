require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianRadialPotentialFields" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   p_gain                = Types.base.VectorXd.new(3)
   max_control_output    = Types.base.VectorXd.new(3)

   for i in (0..2) do
      p_gain[i]                = 1.0
      max_control_output[i]    = 0.3
   end

   controller.field_names        = ["X", "Y", "Z"]
   controller.p_gain             = p_gain
   controller.max_control_output = max_control_output
   controller.influence_distance = 1.0

   controller.configure
   controller.start

   feedback = Types.wbc.CartesianState.new
   feedback.source_frame = "ee"
   feedback.target_frame = "base_link"
   feedback.pose.position[0],feedback.pose.position[1],feedback.pose.position[2] = 0,0,0.1
   feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w = 0,0,0,1

   pot_field = Types.wbc.CartesianState.new
   pot_field.pose.position[0],pot_field.pose.position[1],pot_field.pose.position[2] = 0,0,0
   pot_field.source_frame = "collision_point"
   pot_field.target_frame = "ee"

   feedback_writer        = controller.feedback.writer
   pot_field_writer       = controller.pot_field_centers.writer
   control_output_reader  = controller.control_output.reader
   control_output         = Types.wbc.CartesianState.new

   pot_field_writer.write([pot_field])

   cycle_time = 0.01
   puts("Press Ctrl-C to stop ...")
   while true
      feedback_writer.write(feedback)
      if not control_output_reader.read_new(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do feedback.pose.position[i] += cycle_time*control_output.twist.linear[i] end

      print "Potential Field position:  "
      pot_field.pose.position.data.each do |d| print "#{'%.04f' % d} " end
      print "\nMax control output:        "
      max_control_output.to_a.each do |m| print "#{'%.04f' % m} " end
      print "\nControl Output:            "
      control_output.twist.linear.data.each do |v| print "#{'%.04f' % v} " end
      print "\nActual Position:           "
      feedback.pose.position.data.each do |p| print "#{'%.04f' % p} " end
      print "\n\n"
      sleep(cycle_time)
   end

end
