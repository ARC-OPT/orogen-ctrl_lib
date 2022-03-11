require "orocos"
require "test/unit"

class TestJointLimitAvoidance < Test::Unit::TestCase
    def testSimple
        Orocos.run "ctrl_lib::JointLimitAvoidance" => "controller" do
            Orocos.conf.load_dir("./config")

            controller = Orocos::TaskContext.get "controller"
            Orocos.conf.apply(controller, ["default"])

            assert_nothing_raised(Orocos::StateTransitionFailed){controller.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){controller.start}

            feedback          = Types.base.samples.Joints.new
            feedback.names    = controller.field_names
            state             = Types.base.JointState.new
            state.position    = 0.9
            feedback.elements = [state, state]

            feedback_writer        = controller.feedback.writer
            control_output_reader  = controller.control_output.reader
            activation_reader      = controller.activation.reader
            control_output         = Types.base.commands.Joints.new

            cycle_time = 0.01
            upper = controller.joint_limits.elements[0].max.position
            dist_to_limit = 0
            prev_ctrl_output       = Types.base.commands.Joints.new
            prev_ctrl_output.names = controller.field_names
            state                  = Types.base.JointState.new
            state.speed            = 1e10
            prev_ctrl_output.elements = [state, state]

            while dist_to_limit < controller.influence_distance[0] - 1e-3
                feedback_writer.write(feedback)
                activation = activation_reader.read
                if not control_output_reader.read_new(control_output)
                    sleep(cycle_time)
                    next
                end

                for i in (0..1) do feedback.elements[i].position += cycle_time*control_output.elements[i].speed end

                dist_to_limit = (upper - feedback.elements[0].position).abs

                # 1. Control output has to be smaller than max..control output
                # 2. Control output has to decrease continously
                for i in (0..1) do
                    assert(control_output.elements[i].speed.abs <= controller.max_control_output[i].abs)
                    assert(prev_ctrl_output.elements[i].speed.abs >= control_output.elements[i].speed.abs)
                end

                prev_ctrl_output = control_output.dup
            end # while
        end # Orocos.run
    end # testSimple
end # class
