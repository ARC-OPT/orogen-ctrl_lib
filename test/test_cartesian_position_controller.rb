require "orocos"
require "test/unit"

class TestCartesianPositionController < Test::Unit::TestCase
    def testSimple
        Orocos.run "ctrl_lib::CartesianPositionController" => "controller" do
            Orocos.conf.load_dir("./config")
            controller = Orocos::TaskContext.get "controller"
            Orocos.conf.apply(controller, ["default"])

            assert_nothing_raised(Orocos::StateTransitionFailed){controller.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){controller.start}

            setpoint = Types.base.samples.RigidBodyStateSE3.new
            setpoint.time = Types.base.Time.now
            setpoint.pose.position[0] = 1.0
            setpoint.pose.position[1] = 2.0
            setpoint.pose.position[2] = 3.0
            setpoint.pose.orientation =  Eigen::Quaternion.from_angle_axis(1.571, Eigen::Vector3.UnitZ)

            feedback = Types.base.samples.RigidBodyStateSE3.new
            for i in 0..2 do feedback.pose.position[i] = 0 end
            feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w = 0,0,0,1
            euler = Types.base.Vector3d.new(3)
            euler[0],euler[1],euler[2] = 0,0,0

            setpoint_writer       = controller.setpoint.writer
            feedback_writer       = controller.feedback.writer
            control_output_reader = controller.control_output.reader
            control_output        = Types.base.samples.RigidBodyStateSE3.new

            cycle_time = 0.01
            prev_control_output = Types.base.samples.RigidBodyStateSE3.new
            prev_control_output.twist.linear = Types.base.Vector3d.new(1e10,1e10,1e10)
            prev_control_output.twist.angular = Types.base.Vector3d.new(1e10,1e10,1e10)

            while prev_control_output.twist.linear.norm > 1e-4
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

            # Setpoint must be reached
            assert((setpoint.pose.position - feedback.pose.position).norm < 1e-4)
        end # Orocos.run
    end # testSimple
end # class
