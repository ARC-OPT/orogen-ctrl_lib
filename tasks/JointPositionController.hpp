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
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const base::VectorXd &ctrl_output_raw);
    /** Reset function. Implemented in derived task. This sets the control output to zero by setting setpoint and feedback to the same value.*/
    virtual void reset();

    inline void extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& positions){

        if(joints.elements.size() != joints.names.size()){
            LOG_ERROR("%s: Sizes of names and elements does not match", this->getName().c_str());
            throw std::invalid_argument("Invalid joints vector");
        }

        positions.resize(names.size());
        for(size_t i = 0; i < names.size(); i++){
            const base::JointState& elem = joints.getElementByName(names[i]);
            if(!elem.hasPosition()){
                LOG_ERROR("%s: Element %s does not have a valid position value", this->getName().c_str(), names[i].c_str());
                throw std::invalid_argument("Invalid joints vector");
            }
            positions(i) = elem.position;
        }
    }

    inline void extractVelocities(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& velocities){
        velocities.resize(names.size());
        velocities.setZero();
        for(size_t i = 0; i < names.size(); i++){
            const base::JointState& elem = joints.getElementByName(names[i]);
            if(!elem.hasSpeed()){ // If no speeds are given for an element, disable feed forward term
                velocities.setConstant(0);
                return;
            }
            velocities(i) = elem.speed;
        }
    }

    base::commands::Joints setpoint, control_output;
    base::samples::Joints feedback;
    bool disable_feedback;

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

