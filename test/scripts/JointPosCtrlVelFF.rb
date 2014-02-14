require 'orocos'
require 'vizkit'
require 'pry'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run 'ctrl_lib::JointPosCtrlVelFF' => 'ctrl' do  
    
   ctrl = Orocos::Async.name_service.get 'ctrl'
   Orocos.conf.apply(ctrl, ['default'])

   setpoint_port = ctrl.port("setpoint").writer
   feedback_port = ctrl.port("feedback").writer
   ctrl_out_port = ctrl.port("ctrl_out")

   setpoint = Types::Base::Samples::Joints.new
   feedback = Types::Base::Samples::Joints.new

   state = Types::Base::JointState.new
   state.position = 1.0
   setpoint.elements << state
   setpoint.names << "J_Foot"

   state.position = 0.0
   feedback.elements << state
   feedback.names << "J_Foot"

   
   ctrl.configure
   ctrl.start

   sample_time = 0.1
   setpoint_port.write(setpoint)
   feedback_port.write(feedback)

   sleep(1.0)

   ctrl_out_port.on_data do |ctrl_out| 
       for i in 0..feedback.elements.size()-1
           feedback.elements[i].position = feedback.elements[i].position + ctrl_out.elements[i].speed * sample_time
           puts feedback.elements[i].position
       end
       feedback_port.write(feedback)
   end

   Vizkit.exec
    
end
