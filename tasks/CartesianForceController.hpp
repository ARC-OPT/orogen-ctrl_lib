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

    bool isValid(const base::Wrench &w);
    void invalidate(base::Wrench& w);
    const base::VectorXd wrenchToRaw(const base::samples::Wrench& wrench, base::VectorXd& raw);

    base::samples::Wrench setpoint, feedback;
    base::samples::RigidBodyState control_output;
    base::VectorXd feedback_raw, setpoint_raw;

public:
    CartesianForceController(std::string const& name = "ctrl_lib::CartesianForceController");
    CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianForceController(){}
    bool configureHook(){return CartesianForceControllerBase::configureHook();}
    bool startHook(){return CartesianForceControllerBase::startHook();}
    void updateHook(){CartesianForceControllerBase::updateHook();}
    void errorHook(){CartesianForceControllerBase::errorHook();}
    void stopHook(){CartesianForceControllerBase::stopHook();}
    void cleanupHook(){CartesianForceControllerBase::cleanupHook();}
};
}

#endif

