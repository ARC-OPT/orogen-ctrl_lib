/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANFORCECONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANFORCECONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianForceControllerBase.hpp"
#include <base/samples/Wrench.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace ctrl_lib{

/*! \class CartesianForceController Implementation of a force controller in Cartesian space. See ctrl_lib/ProportionalController.hpp for details*/
class CartesianForceController : public CartesianForceControllerBase
{
    friend class CartesianForceControllerBase;
protected:

    base::samples::Wrench setpoint, feedback;
    base::samples::RigidBodyState control_output;

    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const Eigen::VectorXd &ctrl_output_raw);


public:
    CartesianForceController(std::string const& name = "ctrl_lib::CartesianForceController");
    CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianForceController();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

