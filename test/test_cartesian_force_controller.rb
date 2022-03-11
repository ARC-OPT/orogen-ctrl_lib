require "orocos"
require "test/unit"

class TestCartesianForceController < Test::Unit::TestCase
    def testSimple
        Orocos.run "ctrl_lib::CartesianForceController" => "controller" do
            Orocos.conf.load_dir("./config")
            controller = Orocos::TaskContext.get "controller"
            Orocos.conf.apply(controller, ["default"])

            assert_nothing_raised(Orocos::StateTransitionFailed){controller.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){controller.start}

            setpoint = Types.base.samples.Wrench.new
            setpoint.time = Types.base.Time.now
            setpoint.force[0],setpoint.force[1],setpoint.force[2] = 1.0,2.0,3.0
            setpoint.torque[0],setpoint.torque[1],setpoint.torque[2] = 0.0,0.0,0.0

            feedback = Types.base.samples.Wrench.new
            feedback.force[0],feedback.force[1],feedback.force[2] = 0.0,0.0,0.0
            feedback.torque[0],feedback.torque[1],feedback.torque[2] = 0.0,0.0,0.0

            setpoint_writer      = controller.setpoint.writer
            feedback_writer      = controller.feedback.writer
            control_output_reader = controller.control_output.reader
            control_output        = Types.base.samples.RigidBodyStateSE3.new

            setpoint_writer.write(setpoint)

            cycle_time = 0.01
            prev_control_output = Types.base.samples.RigidBodyStateSE3.new
            prev_control_output.twist.linear = Types.base.Vector3d.new(1e10,1e10,1e10)
            prev_control_output.twist.angular = Types.base.Vector3d.new(1e10,1e10,1e10)

            while prev_control_output.twist.linear.norm > 1e-4
                feedback.time = Types.base.Time.now
                feedback_writer.write(feedback)
                if not control_output_reader.read_new(control_output)
                    sleep(cycle_time)
                    next
                end

                for i in (0..2) do
                    feedback.force[i] += cycle_time*control_output.twist.linear[i]
                    feedback.torque[i] += cycle_time*control_output.twist.angular[i]
                end

                # 1. Control output has to be smaller than max..control output
                # 2. Control output has to decrease continously
                for i in (0..2) do
                    assert(control_output.twist.linear[i].abs <= controller.max_control_output[i].abs)
                    assert(control_output.twist.angular[i].abs <= controller.max_control_output[i+3].abs)
                    assert(control_output.twist.linear[i].abs <= prev_control_output.twist.linear[i].abs + 1e-6)
                    assert(control_output.twist.angular[i].abs <= prev_control_output.twist.angular[i].abs + 1e-6)
                end

                prev_control_output = control_output.dup
            end # while
        end # Orocos.run
    end # testSimple
end # class
