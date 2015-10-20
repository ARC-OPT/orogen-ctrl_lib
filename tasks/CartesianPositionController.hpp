/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianPositionControllerBase.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib {

/*! \class CartesianPositionController Implementation of PositionControlFeedForward in Cartesian space */
class CartesianPositionController : public CartesianPositionControllerBase
{
    friend class CartesianPositionControllerBase;
protected:

    base::samples::RigidBodyState setpoint, control_output, feedback;

    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const base::VectorXd &ctrl_output_raw);
    /** Reset function. Implemented in derived task. This sets the control output to zero by setting setpoint and feedback to the same value.*/
    virtual void reset();
public:
    CartesianPositionController(std::string const& name = "ctrl_lib::CartesianPositionController");
    CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianPositionController();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

