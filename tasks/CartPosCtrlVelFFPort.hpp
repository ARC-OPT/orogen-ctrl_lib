/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTPOSCTRLVELFFPORT_TASK_HPP
#define CTRL_LIB_CARTPOSCTRLVELFFPORT_TASK_HPP

#include "ctrl_lib/CartPosCtrlVelFFPortBase.hpp"

namespace ctrl_lib {


class CartPosCtrlVelFFPort : public CartPosCtrlVelFFPortBase
{
    friend class CartPosCtrlVelFFPortBase;
protected:



public:
    CartPosCtrlVelFFPort(std::string const& name = "ctrl_lib::CartPosCtrlVelFFPort");
    CartPosCtrlVelFFPort(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartPosCtrlVelFFPort();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

