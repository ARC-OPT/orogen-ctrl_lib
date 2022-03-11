require "orocos"
require "test/unit"

class TestJointPositionController < Test::Unit::TestCase
    def testSimple
        Orocos.run "ctrl_lib::JointPositionController" => "controller" do
            Orocos.conf.load_dir("./config")
            controller = Orocos::TaskContext.get "controller"
            Orocos.conf.apply(controller, ["default"])

            assert_nothing_raised(Orocos::StateTransitionFailed){controller.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){controller.start}

            setpoint          = Types.base.commands.Joints.new
            setpoint.names    = controller.field_names
            cmd1              = Types.base.JointState.new
            cmd2              = Types.base.JointState.new
            cmd1.position     = 1.0
            cmd2.position     = 2.0
            setpoint.elements = [cmd1, cmd2]
            setpoint.time     = Types.base.Time.now

            feedback          = Types.base.samples.Joints.new
            feedback.names    = controller.field_names
            state             = Types.base.JointState.new
            state.position    = 0.0
            feedback.elements = [state, state]

            setpoint_writer       = controller.setpoint.writer
            feedback_writer       = controller.feedback.writer
            control_output_reader = controller.control_output.reader
            control_output        = Types.base.commands.Joints.new

            setpoint_writer.write(setpoint)

            prev_control_output          = Types.base.commands.Joints.new
            prev_control_output.names    = controller.field_names
            state                        = Types.base.JointState.new
            state.speed                  = 1e10
            prev_control_output.elements = [state, state]
            cycle_time = 0.01

            while (setpoint.elements[0].position - feedback.elements[0].position).abs > 1e-3 or
                  (setpoint.elements[1].position - feedback.elements[1].position).abs > 1e-3
                feedback.time = Types.base.Time.now
                feedback_writer.write(feedback)
                if not control_output_reader.read_new(control_output)
                    sleep(cycle_time)
                    next
                end

                for i in (0..1) do feedback.elements[i].position += cycle_time*control_output.elements[i].speed end

                # 1. Control output has to be smaller than max..control output
                # 2. Control output has to decrease continously
                for i in (0..1) do
                    assert(control_output.elements[i].speed.abs <= controller.max_control_output[i].abs)
                    assert(control_output.elements[i].speed.abs <= prev_control_output.elements[i].speed.abs + 1e-6)
                end
                prev_control_output = control_output.dup
            end

            # Setpoint must be reached
            for i in (0..1) do
                assert((setpoint.elements[i].position - feedback.elements[i].position).abs <= 1e-3)
            end
        end # Orocos.run
    end # testSimple
end # class
