/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_POTENTIALFIELDSCONTROLLERTASK_TASK_HPP
#define CTRL_LIB_POTENTIALFIELDSCONTROLLERTASK_TASK_HPP

#include "ctrl_lib/PotentialFieldsControllerTaskBase.hpp"
#include <ctrl_lib/PotentialFieldsController.hpp>

namespace ctrl_lib {

class PotentialFieldsControllerTask : public PotentialFieldsControllerTaskBase
{
    friend class PotentialFieldsControllerTaskBase;
protected:
    /** Update all available (dynamic) controller properties*/
    virtual void updateControllerProperties();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback() = 0;
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint() = 0;
    /** Compute output of the controller*/
    virtual const base::VectorXd& updateController();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw) = 0;
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function) = 0;

    PotentialFieldsController* controller;
    std::vector<PotentialFieldInfo> field_infos;
    base::VectorXd tmp;
    double default_influence_distance;
    std::map<std::string, double> influence_distance_map;

    std::map<std::string, double> makeMap(const std::vector<InfluenceDistancePerField>& influence_distance_per_field){
        std::map<std::string, double> tmp_map;
        for(uint i = 0; i < influence_distance_per_field.size(); i++)
            tmp_map[influence_distance_per_field[i].name] = influence_distance_per_field[i].distance;
        return tmp_map;
    }

public:
    PotentialFieldsControllerTask(std::string const& name = "ctrl_lib::PotentialFieldsControllerTask");
    PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~PotentialFieldsControllerTask(){}
    bool configureHook(){return PotentialFieldsControllerTaskBase::configureHook();}
    bool startHook(){return PotentialFieldsControllerTaskBase::startHook();}
    void updateHook(){PotentialFieldsControllerTaskBase::updateHook();}
    void errorHook(){PotentialFieldsControllerTaskBase::errorHook();}
    void stopHook(){PotentialFieldsControllerTaskBase::stopHook();}
    void cleanupHook(){PotentialFieldsControllerTaskBase::cleanupHook();}
};
}

#endif

