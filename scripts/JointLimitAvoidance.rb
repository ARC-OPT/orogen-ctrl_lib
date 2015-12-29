require 'orocos'

Orocos.initialize
Orocos.load_typekit("base")
Orocos.load_typekit("ctrl_lib")

Orocos.run "ctrl_lib::JointLimitAvoidance" => "controller" do

   controller = Orocos::TaskContext.get "controller"

   prop_gain              = Types::Base::VectorXd.new(2)
   max_control_output     = Types::Base::VectorXd.new(2)
   max_influence_distance = Types::Base::VectorXd.new(2)
   dead_zone              = Types::Base::VectorXd.new(2)
   joint_limits           = Types::Base::JointLimits.new
   joint_limits.names     = ["Joint_1", "Joint_2"]
   activation             = Types::CtrlLib::ActivationFunction.new
   for i in (0..1) do
      prop_gain[i]              = 0.01
      max_control_output[i]     = 0.1
      max_influence_distance[i] = 0.3
      range                     = Types::Base::JointLimitRange.new
      range.max.position        = 1.0
      range.min.position        = -1.0
      dead_zone[i]              = 0
      joint_limits.elements << range
   end
   activation.threshold = 0.5
   activation.type = :LINEAR_ACTIVATION

   controller.field_names         = ["Joint_1", "Joint_2"]
   controller.prop_gain           = prop_gain
   controller.dead_zone           = dead_zone
   controller.max_control_output  = max_control_output
   controller.influence_distance  = max_influence_distance
   controller.order               = 1
   controller.joint_limits        = joint_limits
   controller.activation_function = activation
   controller.order               = 1

   controller.configure
   controller.start

   position          = Types::Base::Samples::Joints.new
   position.names    = controller.field_names
   state                    = Types::Base::JointState.new
   state.position           = 0.9
   position.elements = Array[state, state]

   feedback_writer        = controller.feedback.writer
   control_output_reader  = controller.control_output.reader
   activation_reader      = controller.activation.reader
   control_output         = Types::Base::Commands::Joints.new

   cycle_time = 0.01
   puts "Press Ctrl-C to exit..."
   while true
      feedback_writer.write(position)
      activation = activation_reader.read
      if not control_output_reader.read(control_output)
         sleep(cycle_time)
         next
      end

      for i in (0..1) do position.elements[i].position += cycle_time*control_output.elements[i].speed end

      print "Upper limits:            Joint_1: #{'%.04f' % joint_limits.elements[0].max.position}, Joint_2: #{'%.04f' % joint_limits.elements[1].max.position}\n"
      print "Lower limits:            Joint_1: #{'%.04f' % joint_limits.elements[0].min.position}, Joint_2: #{'%.04f' % joint_limits.elements[1].min.position}\n"
      print "Control Output:          Joint_1: #{'%.04f' % control_output.elements[0].speed}, Joint_2: #{'%.04f' % control_output.elements[1].speed}\n"
      print "Activation:              Joint_1: #{'%.04f' % activation[0]}, Joint_2: #{'%.04f' % activation[1]}\n"
      print "Actual Position:         Joint_1: #{'%.04f' % position.elements[0].position}, Joint_2: #{'%.04f' % position.elements[1].position}\n\n"
      sleep(cycle_time)
   end

end
