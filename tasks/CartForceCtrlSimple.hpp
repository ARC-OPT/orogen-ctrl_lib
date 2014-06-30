/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTFORCECTRLSIMPLE_TASK_HPP
#define CTRL_LIB_CARTFORCECTRLSIMPLE_TASK_HPP

#include "ctrl_lib/CartForceCtrlSimpleBase.hpp"
#include <kdl_conversions/KDLConversions.hpp>

namespace ctrl_lib {

class CartForceCtrlSimple : public CartForceCtrlSimpleBase
{
    friend class CartForceCtrlSimpleBase;
protected:
    base::samples::Wrench wrench_ref_, wrench_;
    base::samples::RigidBodyState ft2baseFrame_, ref2baseFrame_, ctrl_out_;
    KDL::Frame ft2baseFrame_kdl_, ref2baseFrame_kdl_;
    KDL::Wrench F_r_, F_;
    base::Vector6d kp_, ctrl_error_, ctrl_out_eigen_;
    base::Vector6d max_ctrl_output_, dead_zone_, activation_;
    base::Time stamp_;

    inline void KDLWrench2Eigen(const KDL::Wrench &in, base::Vector6d &out){
        for(uint i = 0; i < 3; i++){
            out(i) = in.force(i);
            out(i+3) = in.torque(i);
        }
    }
    void validateWrench(const base::samples::Wrench& w){
        for(uint i = 0; i < 3; i++){
            if(base::isNaN(w.force(i)) ||
                    base::isNaN(w.torque(i))){
                LOG_ERROR("An entry of controller input is NaN. Aborting...");
                throw std::invalid_argument("Invalid wrench");
            }
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

