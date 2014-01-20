/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTREPPOTFIELD_TASK_HPP
#define CTRL_LIB_JOINTREPPOTFIELD_TASK_HPP

#include "ctrl_lib/JointRepPotFieldBase.hpp"
#include <ctrl_lib/RepulsivePotentialField.hpp>

namespace ctrl_lib {

class JointRepPotField : public JointRepPotFieldBase
{
    friend class JointRepPotFieldBase;
protected:
    std::vector<RepulsivePotentialField*> rpf_;
    base::samples::Joints feedback_;
    base::commands::Joints ctrl_output_;

public:
    JointRepPotField(std::string const& name = "ctrl_lib::JointRepPotField");
    JointRepPotField(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointRepPotField(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){JointRepPotFieldBase::errorHook();}
    void stopHook(){JointRepPotFieldBase::stopHook();}
    void cleanupHook();
};
}

#endif

