/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTransformationProxy.hpp"

using namespace ctrl_lib;

ControllerTransformationProxy::ControllerTransformationProxy(std::string const& name)
    : ControllerTransformationProxyBase(name){
}

ControllerTransformationProxy::ControllerTransformationProxy(std::string const& name, RTT::ExecutionEngine* engine)
    : ControllerTransformationProxyBase(name, engine){
}

ControllerTransformationProxy::~ControllerTransformationProxy(){
}

bool ControllerTransformationProxy::configureHook(){
    if(!ControllerTransformationProxyBase::configureHook())
         return false;

    source_frame = _source_frame.get();
    target_frame = _target_frame.get();

    return true;
}

void ControllerTransformationProxy::updateHook(){
    ControllerTransformationProxyBase::updateHook();

    if(_source2target.get(base::Time::now(), transform)){
        transform.sourceFrame = source_frame;
        transform.targetFrame = target_frame;
        _transform.write(transform);
    }
}
