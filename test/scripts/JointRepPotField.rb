require 'orocos'
require 'vizkit'
require 'pry'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run 'ctrl_lib::JointRepPotField' => 'pot_field' do  
    
   pot_field = Orocos::Async.name_service.get 'pot_field'
   Orocos.conf.apply(pot_field, ['default'])

   feedback_port = pot_field.port("feedback").writer
   ctrl_out_port = pot_field.port("ctrl_out")

   feedback = Types::Base::Samples::Joints.new
   state = Types::Base::JointState.new
   state.position = 0.1
   feedback.elements << state
   feedback.names << "J_Foot"
	
   pot_field.configure
   pot_field.start

   sleep(1.0)
   sample_time = 0.1
   feedback_port.write(feedback)

   ctrl_out_port.on_data do |ctrl_out| 
       for i in 0..feedback.elements.size()-1
            feedback.elements[i].position = feedback.elements[i].position + ctrl_out.elements[i].speed * sample_time
            puts feedback.elements[i].position
       end
       feedback_port.write(feedback)
   end

   Vizkit.exec
    
end
