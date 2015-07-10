/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/JointPositionControllerBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/Logging.hpp>

namespace ctrl_lib {

/*! \class JointPositionController
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
Implementation of PositionControlFeedForward in joint space. See ctrl_lib/PositionControlFeedForward.hpp for details

     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','ctrl_lib::JointPositionController')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
class JointPositionController : public JointPositionControllerBase
{
    friend class JointPositionControllerBase;
protected:

    std::vector<std::string> jointNames;
    base::commands::Joints setpoint, controlOutput;
    base::samples::Joints feedback;

    virtual bool readSetpoints();
    virtual bool readFeedback();
    virtual void writeControlOutput(const Eigen::VectorXd &y);

    inline void extractPositions(const base::commands::Joints &command, const std::vector<std::string> jointNames, Eigen::VectorXd& positions){
        positions.resize(jointNames.size());
        for(size_t i  = 0; i < jointNames.size(); i++){
            const base::JointState &cmd = command.getElementByName(jointNames[i]);
            if(!cmd.hasPosition()){ //Throw here, since we always need valid positions for this controller
                LOG_ERROR("Element %s has no valid position value!", jointNames[i].c_str());
                throw std::invalid_argument("Invalid position value");
            }
            positions(i) = cmd.position;
        }
    }

    inline void extractSpeeds(const base::commands::Joints &command, const std::vector<std::string> jointNames, Eigen::VectorXd& speeds){
        speeds.resize(jointNames.size());
        for(size_t i = 0; i < jointNames.size(); i++){
            const base::JointState &cmd = command.getElementByName(jointNames[i]);
            if(cmd.hasSpeed())
                speeds(i) = cmd.speed;
            else
                speeds(i) = 0;
        }
    }

public:
    JointPositionController(std::string const& name = "ctrl_lib::JointPositionController");
    JointPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPositionController();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

