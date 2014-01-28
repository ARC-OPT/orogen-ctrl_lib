require 'orocos'
require 'vizkit'
require 'pry'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('../config')
Orocos.transformer.load_conf('../config/transforms.rb')

Orocos.run 'ctrl_lib::CartRepPotField' => 'pot_field' do  
    
   pot_field = Orocos::TaskContext.get 'pot_field'
   Orocos.conf.apply(pot_field, ['default'])

   reader = pot_field.port("ctrl_out").reader

   ctrl_out = Types::Base::Samples::RigidBodyState.new
   rep_field_center = Types::Base::Samples::RigidBodyState.new 
   rep_field_center.sourceFrame = "controlled_in"
   rep_field_center.targetFrame = "setpoint"
   feedback = Types::Base::Samples::RigidBodyState.new
   feedback.sourceFrame = "controlled_in"
   feedback.targetFrame = "controlled_frame"

   rep_field_center.position = Types::Base::Vector3d.new(0,0,0)
   feedback.position = Types::Base::Vector3d.new(0.1, 0.1, 0.1)

   rep_field_center.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)
   feedback.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)

   my_task = Orocos::RubyTaskContext.new("my_task")
   my_task.create_output_port("setpoint", "/base/samples/RigidBodyState")
   my_task.create_output_port("feedback", "/base/samples/RigidBodyState")
   my_task.start

   Orocos.transformer.setup(pot_field)  
   pot_field.configure
   pot_field.start

   sample_time = 0.1
   sleep(2)

   while true do
      if reader.read(ctrl_out) != nil then
         euler = feedback.orientation.to_euler()
         euler = euler + ctrl_out.angular_velocity * sample_time
         feedback.position = feedback.position + ctrl_out.velocity * sample_time
         puts ctrl_out.velocity
      end
      my_task.feedback.write(feedback)
      my_task.setpoint.write(rep_field_center)
      sleep(sample_time)
   end 
    
end
