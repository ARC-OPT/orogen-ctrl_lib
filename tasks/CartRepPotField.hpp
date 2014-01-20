/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTREPPOTFIELD_TASK_HPP
#define CTRL_LIB_CARTREPPOTFIELD_TASK_HPP

#include "ctrl_lib/CartRepPotFieldBase.hpp"
#include <ctrl_lib/RepulsivePotentialField.hpp>

namespace ctrl_lib {


class CartRepPotField : public CartRepPotFieldBase
{
    friend class CartRepPotFieldBase;
protected:
    RepulsivePotentialField* rpf_;
    base::samples::RigidBodyState cur_, center_, ctrl_output_;
public:
    CartRepPotField(std::string const& name = "ctrl_lib::CartRepPotField");
    CartRepPotField(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartRepPotField(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){CartRepPotFieldBase::errorHook();}
    void stopHook(){CartRepPotFieldBase::stopHook();}
    void cleanupHook();
};
}

#endif

