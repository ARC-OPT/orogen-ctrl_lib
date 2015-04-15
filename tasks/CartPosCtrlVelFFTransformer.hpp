/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTPOSCTRLVELFFTRANSFORMER_TASK_HPP
#define CTRL_LIB_CARTPOSCTRLVELFFTRANSFORMER_TASK_HPP

#include "ctrl_lib/CartPosCtrlVelFFTransformerBase.hpp"

namespace ctrl_lib {

class CartPosCtrlVelFFTransformer : public CartPosCtrlVelFFTransformerBase
{
    friend class CartPosCtrlVelFFTransformerBase;
protected:



public:
    CartPosCtrlVelFFTransformer(std::string const& name = "ctrl_lib::CartPosCtrlVelFFTransformer");
    CartPosCtrlVelFFTransformer(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartPosCtrlVelFFTransformer();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

