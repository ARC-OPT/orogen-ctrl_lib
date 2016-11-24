/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTLIMITAVOIDANCE_TASK_HPP
#define CTRL_LIB_JOINTLIMITAVOIDANCE_TASK_HPP

#include "ctrl_lib/JointLimitAvoidanceBase.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib{

class PotentialFieldsController;

/*! \class JointLimitAvoidance Implementation of RadialPotentialFields in joint space. Each joint will have one 1-dimensional potential field.
See ctrl_lib/RadialPotentialField.hpp and ctrl_lib/PotentialFieldsController.hpp for details */
class JointLimitAvoidance : public JointLimitAvoidanceBase
{
    friend class JointLimitAvoidanceBase;

public:
    JointLimitAvoidance(std::string const& name = "ctrl_lib::JointLimitAvoidance");
    JointLimitAvoidance(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointLimitAvoidance(){}
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
    /** Extract positions from joint state*/
    void extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& positions);

    base::JointLimits joint_limits;
    base::samples::Joints feedback;
    base::VectorXd position_raw;
    base::commands::Joints control_output;
    base::VectorXd influence_distance;
    PotentialFieldsController* controller;
    std::vector<PotentialFieldInfo> field_infos;
};
}

#endif

