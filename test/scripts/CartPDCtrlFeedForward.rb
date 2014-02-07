require 'orocos'
require 'vizkit'
require 'pry'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('../config')
Orocos.transformer.load_conf('../config/transforms.rb')

Orocos.run 'ctrl_lib::CartPDCtrlFeedForward' => 'ctrl' do  
    
   ctrl = Orocos::TaskContext.get 'ctrl'
   Orocos.conf.apply(ctrl, ['default'])

   reader = ctrl.port("ctrl_out").reader

   ctrl_out = Types::Base::Samples::RigidBodyState.new
   ctrl_out.position = Types::Base::Vector3d.new(1,0,0)
   ctrl_out.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)
   setpoint = Types::Base::Samples::RigidBodyState.new
   setpoint.sourceFrame = "controlled_in"
   setpoint.targetFrame = "setpoint"
   setpoint.time = Types::Base::Time.now
   feedback = Types::Base::Samples::RigidBodyState.new
   feedback.sourceFrame = "controlled_in"
   feedback.targetFrame = "controlled_frame"
   feedback.time = Types::Base::Time.now

   setpoint.position = Types::Base::Vector3d.new(1,2,3)
   setpoint.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)

   feedback.position = Types::Base::Vector3d.new(0,0,0)
   feedback.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)

  
   my_task = Orocos::RubyTaskContext.new("my_task")
   my_task.create_output_port("setpoint", "/base/samples/RigidBodyState")
   my_task.create_output_port("feedback", "/base/samples/RigidBodyState")
   my_task.start

   Orocos.transformer.setup(ctrl)  
   ctrl.configure
   ctrl.start

   sample_time = 0.1
   sleep(2.0)

   while true do

      if reader.read(ctrl_out) != nil then
         euler = feedback.orientation.to_euler()
         puts euler
         feedback.position = feedback.position + ctrl_out.velocity * sample_time
         euler = euler + ctrl_out.angular_velocity * sample_time
         feedback.orientation.from_euler(euler, 1,1,1)
      end

      feedback.time = Types::Base::Time.now
      setpoint.time = Types::Base::Time.now
      sleep(sample_time)
      my_task.feedback.write(feedback)
      my_task.setpoint.write(setpoint)
   end

end
