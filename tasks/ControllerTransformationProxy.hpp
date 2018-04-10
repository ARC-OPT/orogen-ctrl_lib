/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CONTROLLERTRANSFORMATIONPROXY_TASK_HPP
#define CTRL_LIB_CONTROLLERTRANSFORMATIONPROXY_TASK_HPP

#include "ctrl_lib/ControllerTransformationProxyBase.hpp"

namespace ctrl_lib{

/*! \class Helper task to get Poses from Transformer */
class ControllerTransformationProxy : public ControllerTransformationProxyBase
{
    friend class ControllerTransformationProxyBase;

    public:
        ControllerTransformationProxy(std::string const& name = "ctrl_lib::ControllerTransformationProxy");
        ControllerTransformationProxy(std::string const& name, RTT::ExecutionEngine* engine);
        ~ControllerTransformationProxy();

        bool configureHook();
        bool startHook(){return ControllerTransformationProxyBase::startHook();}
        void updateHook();
        void errorHook(){ControllerTransformationProxyBase::errorHook();}
        void stopHook(){ControllerTransformationProxyBase::stopHook();}
        void cleanupHook(){ControllerTransformationProxyBase::cleanupHook();}

    protected:
        base::samples::RigidBodyState transform;
        std::string source_frame, target_frame;

};
}

#endif

