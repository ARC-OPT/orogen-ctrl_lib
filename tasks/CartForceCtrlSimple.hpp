/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTFORCECTRLSIMPLE_TASK_HPP
#define CTRL_LIB_CARTFORCECTRLSIMPLE_TASK_HPP

#include "ctrl_lib/CartForceCtrlSimpleBase.hpp"
#include <kdl_conversions/KDLConversions.hpp>

namespace ctrl_lib {

/** Cartesian Force Controller with velocity output ctrl_out = kp * (wrench_ref - wrench)  */
class CartForceCtrlSimple : public CartForceCtrlSimpleBase
{
    friend class CartForceCtrlSimpleBase;
protected:
    base::samples::Wrench wrench_ref_, wrench_;
    base::samples::RigidBodyState ft2refFrame_, ctrl_out_;
    KDL::Frame ft2refFrame_kdl_;
    KDL::Wrench F_r_, F_;
    base::Vector6d kp_, ctrl_error_, ctrl_out_eigen_;
    base::VectorXd max_ctrl_output_;
    base::Time stamp_;
    base::Vector6d contact_threshold_;

    inline void KDLWrench2Eigen(const KDL::Wrench &in, base::Vector6d &out){
        for(uint i = 0; i < 3; i++){
            out(i) = in.force(i);
            out(i+3) = in.torque(i);
        }
    }

public:
    CartForceCtrlSimple(std::string const& name = "ctrl_lib::CartForceCtrlSimple");
    CartForceCtrlSimple(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartForceCtrlSimple();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

