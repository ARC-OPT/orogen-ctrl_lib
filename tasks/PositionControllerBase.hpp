/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_POSITIONCONTROLLERBASE_TASK_HPP
#define CTRL_LIB_POSITIONCONTROLLERBASE_TASK_HPP

#include "ctrl_lib/PositionControllerBaseBase.hpp"

namespace ctrl_lib {

class PositionControlFeedForward;

/*! \class PositionControllerBase
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
ctrlOutput: y = ar + kd(vr - v) + kp(xr - x)

     where: abs(xr - x) >= eMin and y <= yMax

     with:  x, xr - actual and reference position
            v, vr - actual and reference velocity
            ar    - Reference acceleration
            eMin  - Minimum control error (dead zone)
            yMax  - Maximum control output (saturation)
            kp    - Proportional gain
            kd    - Derivative gain

     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','ctrl_lib::PositionControllerBase')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
class PositionControllerBase : public PositionControllerBaseBase
{
    friend class PositionControllerBaseBase;
protected:

    Eigen::VectorXd x, v;
    Eigen::VectorXd xr, vr, ar;
    Eigen::VectorXd y;

    PositionControlFeedForward* controller;

    virtual bool readSetpoints() = 0;
    virtual bool readFeedback() = 0;
    virtual void sendControlOutput() = 0;

public:
    PositionControllerBase(std::string const& name = "ctrl_lib::PositionControllerBase");
    PositionControllerBase(std::string const& name, RTT::ExecutionEngine* engine);
    ~PositionControllerBase();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

