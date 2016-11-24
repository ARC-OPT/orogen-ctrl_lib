/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANFORCECONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANFORCECONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianForceControllerBase.hpp"
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace ctrl_lib{

class ProportionalFeedForwardController;

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
    virtual const base::VectorXd& updateController();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw);
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);

    bool isValid(const base::Wrench &w);
    void invalidate(base::Wrench& w);
    const base::VectorXd wrenchToRaw(const base::samples::Wrench& wrench, base::VectorXd& raw);

    base::samples::Wrench setpoint, feedback;
    base::samples::RigidBodyState control_output;
    base::VectorXd feedback_raw, setpoint_raw;
    ProportionalFeedForwardController* controller;

};
}

#endif

