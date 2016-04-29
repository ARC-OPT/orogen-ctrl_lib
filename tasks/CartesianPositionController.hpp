/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianPositionControllerBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace ctrl_lib {

/*! \class CartesianPositionController Implementation of PositionControlFeedForward in Cartesian space */
class CartesianPositionController : public CartesianPositionControllerBase
{
    friend class CartesianPositionControllerBase;
protected:

    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw);
    /** Reset setpoint to actual value. Control output will be approx. zero after that action*/
    virtual void reset();
    /** Clear setpoint. No control output will be written after that*/
    virtual void clearSetpoint();

    void setControlInput();

    base::samples::RigidBodyState setpoint, control_output, feedback;
    base::VectorXd setpoint_raw, feedback_raw, feedforward_raw;

public:
    CartesianPositionController(std::string const& name = "ctrl_lib::CartesianPositionController");
    CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianPositionController(){}
    bool configureHook(){return CartesianPositionControllerBase::configureHook();}
    bool startHook(){return CartesianPositionControllerBase::startHook();}
    void updateHook(){CartesianPositionControllerBase::updateHook();}
    void errorHook(){CartesianPositionControllerBase::errorHook();}
    void stopHook(){CartesianPositionControllerBase::stopHook();}
    void cleanupHook(){CartesianPositionControllerBase::cleanupHook();}
};
}

#endif

