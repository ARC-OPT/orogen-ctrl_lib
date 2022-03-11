require "orocos"
require "test/unit"

class TestCartesianRadialPotentialFields < Test::Unit::TestCase
    def testSimple
        Orocos.run "ctrl_lib::CartesianRadialPotentialFields" => "controller" do
            Orocos.conf.load_dir("./config")
            controller = Orocos::TaskContext.get "controller"
            Orocos.conf.apply(controller, ["default"])

            assert_nothing_raised(Orocos::StateTransitionFailed){controller.configure}
            assert_nothing_raised(Orocos::StateTransitionFailed){controller.start}

            feedback = Types.base.samples.RigidBodyStateSE3.new
            feedback.frame_id = "base_link"
            feedback.pose.position[0],feedback.pose.position[1],feedback.pose.position[2] = 0,0,0.1
            feedback.pose.orientation.x,feedback.pose.orientation.y,feedback.pose.orientation.z,feedback.pose.orientation.w = 0,0,0,1

            pot_field = Types.base.samples.RigidBodyState.new
            pot_field.position[0],pot_field.position[1],pot_field.position[2] = 0,0,0
            pot_field.targetFrame = "base_link"

            feedback_writer        = controller.feedback.writer
            pot_field_writer       = controller.pot_field_centers.writer
            control_output_reader  = controller.control_output.reader
            control_output         = Types.base.samples.RigidBodyStateSE3.new

            pot_field_writer.write([pot_field])
            prev_control_output = Types.base.samples.RigidBodyStateSE3.new
            prev_control_output.twist.linear = Types.base.Vector3d.new(1e10,1e10,1e10)
            prev_control_output.twist.angular = Types.base.Vector3d.new(1e10,1e10,1e10)

            prev_feedback = feedback.dup

            cycle_time = 0.01
            while control_output.twist.linear[2] > 0
                feedback_writer.write(feedback)
                if not control_output_reader.read_new(control_output)
                    sleep(cycle_time)
                    next
                end

                for i in (0..2) do feedback.pose.position[i] += cycle_time*control_output.twist.linear[i] end

                # 1. Control output has to be smaller than max..control output
                # 2. Control output has to decrease continously
                # 3. Robot distance to pot. field has to increase
                for i in (0..2) do
                    assert(control_output.twist.linear[i].abs <= controller.max_control_output[i].abs)
                    assert(control_output.twist.linear[i].abs <= prev_control_output.twist.linear[i].abs + 1e-6)
                    assert(prev_feedback.pose.position[i] <=  feedback.pose.position[i])
                end

                prev_feedback = feedback.dup
            end # while
        end # Orocos.run
    end # testSimple
end # class
