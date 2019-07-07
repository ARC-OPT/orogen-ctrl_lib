require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")
Orocos.load_typekit("ctrl_lib")

Orocos.run "ctrl_lib::JointTorqueController" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   pid                = Types.ctrl_lib.PIDCtrlParams.new
   pid.p_gain         = Types.base.VectorXd.new(2)
   pid.i_gain         = Types.base.VectorXd.new(2)
   pid.d_gain         = Types.base.VectorXd.new(2)
   pid.windup         = Types.base.VectorXd.new(2)
   dead_zone          = Types.base.VectorXd.new(2)
   max_control_output = Types.base.VectorXd.new(2)

   pid.p_gain[0],pid.p_gain[1]                         = 1.0,1.0
   pid.i_gain[0],pid.i_gain[1]                         = 0,0
   pid.d_gain[0],pid.d_gain[1]                         = 0,0
   dead_zone[0],dead_zone[1]                   = 0.05,0.1
   max_control_output[0],max_control_output[1] = 1e10,1e10

   controller.field_names        = ["Joint_1", "Joint_2"]
   controller.pid_params         = pid
   controller.dead_zone          = dead_zone
   controller.max_control_output = max_control_output

   controller.configure
   controller.start

   setpoint          = Types.base.commands.Joints.new
   setpoint.names    = controller.field_names
   cmd1              = Types.base.JointState.new
   cmd2              = Types.base.JointState.new
   cmd1.effort       = 1.0
   cmd2.effort       = 2.0
   setpoint.elements = Array[cmd1, cmd2]

   feedback          = Types.base.samples.Joints.new
   feedback.names    = controller.field_names
   state             = Types.base.JointState.new
   state.effort      = 0.0
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

      for i in (0..1) do feedback.elements[i].effort += cycle_time*control_output.elements[i].speed end

      print "Goal Effort:   Joint_1: #{'%.04f' % setpoint.elements[0].effort}, Joint_2: #{'%.04f' % setpoint.elements[1].effort}\n"
      print "Control Output:  Joint_1: #{'%.04f' % control_output.elements[0].speed}, Joint_2: #{'%.04f' % control_output.elements[1].speed}\n"
      print "Actual Effort: Joint_1: #{'%.04f' % feedback.elements[0].effort}, Joint_2: #{'%.04f' % feedback.elements[1].effort}\n\n"
      sleep(cycle_time)
   end
end
