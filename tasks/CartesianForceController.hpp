/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANFORCECONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANFORCECONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianForceControllerBase.hpp"
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

namespace ctrl_lib{

class CartesianForcePIDController;

/*! \class CartesianForceController Implementation of a force controller in Cartesian space. See ctrl_lib/ProportionalController.hpp for details*/
class CartesianForceController : public CartesianForceControllerBase
{
    friend class CartesianForceControllerBase;

public:
    CartesianForceController(std::string const& name = "ctrl_lib::CartesianForceController");
    CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianForceController(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();

protected:
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Compute output of the controller*/
    virtual void updateController();
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);


    bool isValid(const base::Wrench &w){
        return !base::isNaN(w.force(0)) && !base::isNaN(w.force(1)) && !base::isNaN(w.force(2)) &&
               !base::isNaN(w.torque(0)) && !base::isNaN(w.torque(1)) && !base::isNaN(w.torque(2));
    }

    const base::VectorXd wrenchToRaw(const base::samples::Wrench& wrench, base::VectorXd& raw){
        raw.resize(6);
        raw.segment(0,3) = wrench.force;
        raw.segment(3,3) = wrench.torque;
        return raw;
    }

    base::samples::Wrench setpoint, feedback;
    base::VectorXd setpoint_raw, feedback_raw, ctrl_output_raw;
    base::samples::RigidBodyStateSE3 control_output;
    CartesianForcePIDController* controller;

};
}

#endif

