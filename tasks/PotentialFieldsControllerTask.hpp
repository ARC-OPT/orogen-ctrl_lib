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

public:
    PotentialFieldsControllerTask(std::string const& name = "ctrl_lib::PotentialFieldsControllerTask");
    PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~PotentialFieldsControllerTask();
    bool configureHook(){return PotentialFieldsControllerTaskBase::configureHook();}
    bool startHook();
    void updateHook(){PotentialFieldsControllerTaskBase::updateHook();}
    void errorHook(){PotentialFieldsControllerTaskBase::errorHook();}
    void stopHook(){PotentialFieldsControllerTaskBase::stopHook();}
    void cleanupHook();
};
}

#endif

