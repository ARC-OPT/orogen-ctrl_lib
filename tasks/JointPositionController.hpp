/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/JointPositionControllerBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/Logging.hpp>

namespace ctrl_lib {

/*! \class JointPositionController Implementation of PositionControlFeedForward in joint space. See ctrl_lib/PositionControlFeedForward.hpp for details*/
class JointPositionController : public JointPositionControllerBase
{
    friend class JointPositionControllerBase;
protected:

    base::commands::Joints setpoint, control_output;
    base::samples::Joints feedback;

    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const Eigen::VectorXd &ctrl_output_raw);

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

