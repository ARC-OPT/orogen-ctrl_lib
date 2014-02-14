/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPosCtrlVelFF.hpp"
#include <base/Eigen.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl/frames_io.hpp>

using namespace ctrl_lib;
using namespace std;

CartPosCtrlVelFF::CartPosCtrlVelFF(std::string const& name)
    : CartPosCtrlVelFFBase(name)
{
    p_ctrl_ = 0;
}

CartPosCtrlVelFF::CartPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPosCtrlVelFFBase(name, engine)
{
    p_ctrl_ = 0;
}


bool CartPosCtrlVelFF::configureHook()
{
    if (! CartPosCtrlVelFFBase::configureHook())
        return false;

    p_ctrl_ = new PDCtrlFeedForward(6);
    p_ctrl_->kp_ = _kp.get();
    p_ctrl_->v_max_ = _max_ctrl_out.get();

    LOG_DEBUG("Saturation Values: ");
    LOG_DEBUG("v_max: %f %f %f",  p_ctrl_->v_max_(0), p_ctrl_->v_max_(1), p_ctrl_->v_max_(2));
    LOG_DEBUG("v_rot_max: %f %f %f",  p_ctrl_->v_max_(3), p_ctrl_->v_max_(4), p_ctrl_->v_max_(5));

    return true;
}

void CartPosCtrlVelFF::updateHook()
{
    CartPosCtrlVelFFBase::updateHook();

    //Get Transforms:
    _controlled_in2setpoint.get(base::Time::now(), ref_);
    _controlled_in2controlled_frame.get(base::Time::now(), cur_);

    KDL::Frame ref_kdl, cur_kdl;
    kdl_conversions::RigidBodyState2KDL(ref_, ref_kdl);
    kdl_conversions::RigidBodyState2KDL(cur_, cur_kdl);

    if(!ref_.hasValidPosition() ||
            !ref_.hasValidOrientation())
    {
        if((base::Time::now() - stamp_).toSeconds() > 1)
        {
            LOG_DEBUG("Transform between controlled_in and setpoint has no valid position and/or orientation");
            LOG_DEBUG("Norm of quaternion is %f", ref_.orientation.squaredNorm());
            stamp_ = base::Time::now();
        }
        return;
    }

    if(!cur_.hasValidPosition() ||
       !cur_.hasValidOrientation())
    {
        if((base::Time::now() - stamp_).toSeconds() > 1)
        {
            stamp_ = base::Time::now();
            LOG_DEBUG("Transform between controlled_in and controlled_frame has no valid position and/or orientation");
            LOG_DEBUG("Norm of quaternion is %f", ref_.orientation.squaredNorm());
        }
        return;
    }

    //Use KDL::Diff to get a non-singular orientation error
    KDL::Twist diff = KDL::diff(cur_kdl, ref_kdl);

    //Convert to eigen: Use diff as reference value and set current vale to zero
    p_ctrl_->x_r_.segment(0,3) << diff.vel(0), diff.vel(1), diff.vel(2);
    p_ctrl_->x_r_.segment(3,3) << diff.rot(0), diff.rot(1), diff.rot(2);
    p_ctrl_->x_.setZero();

    if(ref_.hasValidVelocity())
        p_ctrl_->v_r_.segment(0,3) = ref_.velocity;
    if(ref_.hasValidAngularVelocity())
        p_ctrl_->v_r_.segment(3,3) = ref_.angular_velocity;

    //Read new pid values
    if(_kp_values.read(kp_) == RTT::NewData)
        p_ctrl_->kp_ = kp_;

    p_ctrl_->updateCtrlOutput();

    //Convert to RigidBodyState*/
    ctrl_output_.time = base::Time::now();
    ctrl_output_.velocity = p_ctrl_->v_ctrl_out_.segment(0,3);
    ctrl_output_.angular_velocity = p_ctrl_->v_ctrl_out_.segment(3,3);
    _ctrl_out.write(ctrl_output_);

    //Write debug data
    ctrl_error_.velocity = (p_ctrl_->x_r_ - p_ctrl_->x_).segment(0,3);
    ctrl_error_.angular_velocity = (p_ctrl_->x_r_ - p_ctrl_->x_).segment(3,3);
    _control_error.write(ctrl_error_);
}

void CartPosCtrlVelFF::cleanupHook(){
    CartPosCtrlVelFFBase::cleanupHook();

    delete p_ctrl_;
    p_ctrl_ = 0;
}
