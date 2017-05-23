/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_CARTESIANPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/CartesianPositionControllerBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace ctrl_lib {

class ProportionalFeedForwardController;

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
    virtual const base::VectorXd& updateController();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw);
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);

    void setControlInput();

    void pose_diff(const base::samples::RigidBodyState& a, const base::samples::RigidBodyState& b, const double dt, base::VectorXd& twist){
        pose_diff(a.getTransform(), b.getTransform(), dt, twist);
    }

    void pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b, const double dt, base::VectorXd& twist){

        Eigen::Matrix3d rot_mat = a.rotation().inverse() * b.rotation();
        Eigen::AngleAxisd angle_axis;
        angle_axis.fromRotationMatrix(rot_mat);

        twist.segment(0,3) = (b.translation() - a.translation())/dt;
        twist.segment(3,3) = a.rotation() * (angle_axis.axis() * angle_axis.angle())/dt;
    }

    base::samples::RigidBodyState setpoint, control_output, feedback;
    base::VectorXd setpoint_raw, feedback_raw, feedforward_raw;
    ProportionalFeedForwardController* controller;
};
}

#endif

