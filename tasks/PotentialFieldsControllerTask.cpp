/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PotentialFieldsControllerTask.hpp"

using namespace ctrl_lib;

PotentialFieldsControllerTask::PotentialFieldsControllerTask(std::string const& name)
    : PotentialFieldsControllerTaskBase(name){
}

PotentialFieldsControllerTask::PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PotentialFieldsControllerTaskBase(name, engine){
}

void PotentialFieldsControllerTask::updateControllerProperties(){
    controller->setPropGain(_prop_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());

    _current_prop_gain.write(controller->getPropGain());
    _current_max_control_output.write(controller->getMaxControlOutput());
}

const base::VectorXd& PotentialFieldsControllerTask::updateController(){
    const base::VectorXd& control_output = controller->update();

    const std::vector<PotentialField*> fields = controller->getFields();
    field_infos.resize(fields.size());
    base::VectorXd euclidean_distance(fields.size());
    for(uint i = 0; i < fields.size(); i++){
        field_infos[i].fromPotentialField(fields[i]);
        euclidean_distance(i) = field_infos[i].euclidean_distance;
    }

    _field_infos.write(field_infos);
    _euclidean_distance.write(euclidean_distance);

    return control_output;
}
