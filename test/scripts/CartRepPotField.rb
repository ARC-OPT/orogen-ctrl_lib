require 'orocos'
require 'vizkit'
require 'pry'

include Orocos

Orocos.initialize
Orocos.conf.load_dir('../config')

Orocos.run 'ctrl_lib::CartRepPotField' => 'pot_field' do  
    
   pot_field = Orocos::Async.name_service.get 'pot_field'
   Orocos.conf.apply(pot_field, ['default'])

   rep_field_center_port = pot_field.port("rep_field_center").writer
   controlled_frame_port = pot_field.port("controlled_frame").writer
   ctrl_out_port = pot_field.port("ctrl_out")

   rep_field_center = Types::Base::Samples::RigidBodyState.new
   controlled_frame = Types::Base::Samples::RigidBodyState.new

   rep_field_center.position = Types::Base::Vector3d.new(0,0,0)
   controlled_frame.position = Types::Base::Vector3d.new(0.5, 0.5, 0.5)

   rep_field_center.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)
   controlled_frame.orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)

   pot_field.configure
   pot_field.start

   sleep(3.0)
   sample_time = 0.1
   rep_field_center_port.write(rep_field_center)
   controlled_frame_port.write(controlled_frame)


   ctrl_out_port.on_data do |ctrl_out| 
       controlled_frame.position = controlled_frame.position + ctrl_out.velocity * sample_time
       controlled_frame_port.write(controlled_frame)
   end

   Vizkit.exec
    
end
