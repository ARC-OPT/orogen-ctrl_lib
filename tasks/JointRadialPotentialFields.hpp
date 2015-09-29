/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTRADIALPOTENTIALFIELDS_TASK_HPP
#define CTRL_LIB_JOINTRADIALPOTENTIALFIELDS_TASK_HPP

#include "ctrl_lib/JointRadialPotentialFieldsBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/Logging.hpp>

namespace ctrl_lib {

/*! \class JointRadialPotentialFields Implementation of RadialPotentialFields in joint space. Each joint will have its own potential field. See ctrl_lib/RadialPotentialField.hpp and
ctrl_lib/MultiPotentialFields.hpp for details */
class JointRadialPotentialFields : public JointRadialPotentialFieldsBase
{
    friend class JointRadialPotentialFieldsBase;
protected:

    /** Read actual position. Return false if there is no valid actual position*/
    virtual bool readActualPosition();
    /** Read potential field centers. Return false if there are no valid potential centers*/
    virtual bool readPotFieldCenters();
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const Eigen::VectorXd &ctrl_output_raw);

    /** Set new positions for the potential fields*/
    void setPotFieldCenters(const base::commands::Joints& centers);

    base::commands::Joints pot_field_centers;
    base::samples::Joints actual_position;
    base::commands::Joints control_output;
    Eigen::VectorXd position;

    inline void extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, Eigen::VectorXd& positions){

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

public:
    JointRadialPotentialFields(std::string const& name = "ctrl_lib::JointRadialPotentialFields");
    JointRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointRadialPotentialFields();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

