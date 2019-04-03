require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")
Orocos.load_typekit("ctrl_lib")

Orocos.run "ctrl_lib::JointLimitAvoidance" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   p_gain                 = Types.base.VectorXd.new(2)
   max_control_output     = Types.base.VectorXd.new(2)
   influence_distance     = Types.base.VectorXd.new(2)
   joint_limits           = Types.base.JointLimits.new
   joint_limits.names     = ["Joint_1", "Joint_2"]
   activation             = Types.ctrl_lib.ActivationFunction.new
   for i in (0..1) do
      p_gain[i]                 = 1.0
      max_control_output[i]     = 0.3
      influence_distance[i]     = 0.3
      range                     = Types.base.JointLimitRange.new
      range.max.position        = 1.0
      range.min.position        = -1.0
      joint_limits.elements << range
   end
   activation.threshold = 0.5
   activation.type = :LINEAR_ACTIVATION

   controller.field_names         = ["Joint_1", "Joint_2"]
   controller.p_gain           = p_gain
   controller.max_control_output  = max_control_output
   controller.influence_distance  = influence_distance
   controller.joint_limits        = joint_limits
   controller.activation_function = activation

   controller.configure
   controller.start

   position          = Types.base.samples.Joints.new
   position.names    = controller.field_names
   state             = Types.base.JointState.new
   state.position    = 0.9
   position.elements = Array[state, state]

   feedback_writer        = controller.feedback.writer
   control_output_reader  = controller.control_output.reader
   activation_reader      = controller.activation.reader
   control_output         = Types.base.commands.Joints.new

   cycle_time = 0.01
   puts "Press Ctrl-C to exit..."
   while true
      feedback_writer.write(position)
      activation = activation_reader.read
      if not control_output_reader.read_new(control_output)
         puts "Waiting for ctrl output"
         sleep(cycle_time)
         next
      end

      for i in (0..1) do position.elements[i].position += cycle_time*control_output.elements[i].speed end

      print "Upper limits:            Joint_1: #{'%.04f' % joint_limits.elements[0].max.position}, Joint_2: #{'%.04f' % joint_limits.elements[1].max.position}\n"
      print "Lower limits:            Joint_1: #{'%.04f' % joint_limits.elements[0].min.position}, Joint_2: #{'%.04f' % joint_limits.elements[1].min.position}\n"
      print "Actual Position:         Joint_1: #{'%.04f' % position.elements[0].position}, Joint_2: #{'%.04f' % position.elements[1].position}\n"
      print "Influence Distance       Joint_1: #{'%.04f' % influence_distance[0]}, Joint_2: #{'%.04f' % influence_distance[1]}\n"
      print "Control Output:          Joint_1: #{'%.04f' % control_output.elements[0].speed}, Joint_2: #{'%.04f' % control_output.elements[1].speed}\n"
      print "Activation:              Joint_1: #{'%.04f' % activation[0]}, Joint_2: #{'%.04f' % activation[1]}\n\n"
      sleep(cycle_time)
   end

end
