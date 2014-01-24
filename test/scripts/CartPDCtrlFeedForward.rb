require 'orocos'
require 'vizkit'
require 'pry'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run 'ctrl_lib::CartPDCtrlFeedForward' => 'ctrl' do  
    
   ctrl = Orocos::Async.name_service.get 'ctrl'
   Orocos.conf.apply(ctrl, ['default'])

   setpoint_port = ctrl.port("setpoint").writer
   feedback_port = ctrl.port("feedback").writer
   ctrl_out_port = ctrl.port("ctrl_out")

   setpoint = Types::Base::Samples::RigidBodyState.new
   feedback = Types::Base::Samples::RigidBodyState.new

   setpoint.position = Types::Base::Vector3d.new(1,0,0)
   feedback.position = Types::Base::Vector3d.new(0,0,0)

   setpoint.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)
   feedback.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)

   ctrl.configure
   ctrl.start

   sample_time = 0.1
   setpoint_port.write(setpoint)
   feedback_port.write(feedback)

   ctrl_out_port.on_data do |ctrl_out| 
       feedback.position = feedback.position + ctrl_out.velocity * sample_time
       feedback_port.write(feedback)
   end

   Vizkit.exec
    
end
