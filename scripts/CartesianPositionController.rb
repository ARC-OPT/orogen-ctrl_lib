require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::CartesianPositionController" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   p_gain             = Types.base.VectorXd.new(6)
   d_gain             = Types.base.VectorXd.new(6)
   dead_zone          = Types.base.VectorXd.new(6)
   max_control_output = Types.base.VectorXd.new(6)

   for i in (0..5) do
      p_gain[i]             = 1.0
      d_gain[i]             = 0.0
      dead_zone[i]          = 0.01
      max_control_output[i] = 0.5
   end

   controller.field_names        = ["X", "Y", "Z", "rotZ", "rotY", "rotZ"]
   controller.p_gain             = p_gain
   controller.d_gain             = d_gain
   controller.dead_zone          = dead_zone
   controller.max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint = Types.wbc.CartesianState.new
   setpoint.time = Types.base.Time.now
   setpoint.pose.position[0] = 1.0
   setpoint.pose.position[1] = 2.0
   setpoint.pose.position[2] = 3.0
   setpoint.pose.orientation =  Eigen::Quaternion.from_angle_axis(1.571, Eigen::Vector3.UnitZ)

   feedback = Types.wbc.CartesianState.new
   for i in 0..2 do feedback.pose.position[i] = 0 end
   feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w = 0,0,0,1
   euler = Types.base.Vector3d.new(3)
   euler[0],euler[1],euler[2] = 0,0,0

   setpoint_writer       = controller.setpoint.writer
   feedback_writer       = controller.feedback.writer
   control_output_reader = controller.control_output.reader
   control_output        = Types.wbc.CartesianState.new

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true
      feedback.time = Types.base.Time.now
      feedback_writer.write(feedback)
      setpoint_writer.write(setpoint)
      if not control_output_reader.read_new(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..2) do
         feedback.pose.position[i] += cycle_time*control_output.twist.linear[i]
         euler[i] += cycle_time*control_output.twist.angular[i]
      end
      feedback.pose.orientation.from_euler(euler,0,1,2)

      print "Goal Pose (x/y/z/rx/ry/rz): "
      setpoint.pose.position.data.each do |p| print "#{'%.04f' % p} " end
      setpoint.pose.orientation.to_euler.data.each do |e| print "#{'%.04f' % e} " end

      print "\nMax control output:         "
      max_control_output.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nDead zone:                  "
      dead_zone.to_a.each do |m| print "#{'%.04f' % m} " end

      print "\nControl Output:             "
      control_output.twist.linear.data.each  do |v| print "#{'%.04f' % v} " end
      control_output.twist.angular.data.each do |v| print "#{'%.04f' % v} " end

      print "\nActual Position:            "
      feedback.pose.position.data.each do |p| print "#{'%.04f' % p} " end
      feedback.pose.orientation.to_euler.data.each do |e| print "#{'%.04f' % e} " end
      print "\n\n"
      sleep(cycle_time)
   end
end
