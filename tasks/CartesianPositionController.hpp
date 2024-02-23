/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianPositionControllerBase.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

namespace ctrl_lib {

class CartesianPosPDController;

/*! \class CartesianPositionController Implementation of PositionControlFeedForward in Cartesian space */
class CartesianPositionController : public CartesianPositionControllerBase
{
    friend class CartesianPositionControllerBase;

public:
    CartesianPositionController(std::string const& name = "ctrl_lib::CartesianPositionController");
    CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianPositionController(){}
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

    base::samples::RigidBodyStateSE3 setpoint, control_output, feedback;
    wbc::CartesianPosPDController* controller;
};
}

#endif

