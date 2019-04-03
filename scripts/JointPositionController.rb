require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")

Orocos.run "ctrl_lib::JointPositionController" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   p_gain             = Types.base.VectorXd.new(2)
   dead_zone          = Types.base.VectorXd.new(2)
   max_control_output = Types.base.VectorXd.new(2)
   d_gain             = Types.base.VectorXd.new(2)

   p_gain[0],p_gain[1]                         = 1.0,1.0
   dead_zone[0],dead_zone[1]                   = 0.05,0.1
   d_gain[0],d_gain[1]                         = 0,0
   max_control_output[0],max_control_output[1] = 1e10,1e10

   controller.field_names        = ["Joint_1", "Joint_2"]
   controller.p_gain             = p_gain
   controller.d_gain             = d_gain
   controller.dead_zone          = dead_zone
   controller.max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint          = Types.base.commands.Joints.new
   setpoint.names    = controller.field_names
   cmd1              = Types.base.JointState.new
   cmd2              = Types.base.JointState.new
   cmd1.position     = 1.0
   cmd2.position     = 2.0
   cmd1.speed        = 0.1
   setpoint.elements = Array[cmd1, cmd2]

   feedback          = Types.base.samples.Joints.new
   feedback.names    = controller.field_names
   state             = Types.base.JointState.new
   state.position    = 0.0
   feedback.elements = Array[state, state]

   setpoint_writer      = controller.setpoint.writer
   feedback_writer      = controller.feedback.writer
   control_output_reader = controller.control_output.reader
   control_output        = Types.base.commands.Joints.new

   setpoint.time = Types.base.Time.now
   setpoint_writer.write(setpoint)

   cycle_time = 0.01
   puts "Press Ctrl-C to stop ..."
   while true
      feedback.time = Types.base.Time.now
      feedback_writer.write(feedback)
      if not control_output_reader.read_new(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..1) do feedback.elements[i].position += cycle_time*control_output.elements[i].speed end

      print "Goal Position:   Joint_1: #{'%.04f' % setpoint.elements[0].position}, Joint_2: #{'%.04f' % setpoint.elements[1].position}\n"
      print "Control Output:  Joint_1: #{'%.04f' % control_output.elements[0].speed}, Joint_2: #{'%.04f' % control_output.elements[1].speed}\n"
      print "Actual Position: Joint_1: #{'%.04f' % feedback.elements[0].position}, Joint_2: #{'%.04f' % feedback.elements[1].position}\n\n"
      sleep(cycle_time)
   end
end
