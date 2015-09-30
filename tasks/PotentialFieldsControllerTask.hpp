/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_POTENTIALFIELDSCONTROLLERTASK_TASK_HPP
#define CTRL_LIB_POTENTIALFIELDSCONTROLLERTASK_TASK_HPP

#include "ctrl_lib/PotentialFieldsControllerTaskBase.hpp"
#include <ctrl_lib/PotentialFieldsController.hpp>
#include "ctrl_libTypes.hpp"
#include <base/Logging.hpp>

namespace ctrl_lib{
class PotentialFieldsControllerTask : public PotentialFieldsControllerTaskBase
{
    friend class PotentialFieldsControllerTaskBase;
protected:

    /** Read actual position. Return false if there is no valid actual position*/
    virtual bool readActualPosition() = 0;
    /** Read potential field centers. Return false if there are no valid potential centers*/
    virtual bool readPotFieldCenters() = 0;
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const Eigen::VectorXd &ctrl_output_raw) = 0;

    PotentialFieldsController* controller;
    std::vector<std::string> field_names;
    std::vector<PotentialFieldInfo> field_infos;
    Eigen::VectorXd control_output_raw;
    bool has_pot_field_centers, has_position;

    void setInfluenceDistance(const base::VectorXd &distance){

        assert(controller != 0);

        // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
        // only do sth. if the size is not zero
        if(distance.size() != 0){

            if(distance.size() != (int)controller->fields.size()){
                LOG_ERROR("%s: Max. Influence Distance vector has size %s, but field names size is ",
                          this->getName().c_str(), distance.size(), controller->fields.size());
                throw std::invalid_argument("Invalid influence distance");
            }
            for(size_t i = 0; i < controller->fields.size(); i++)
                controller->fields[i]->influence_distance = distance(i);
        }
    }

public:
    PotentialFieldsControllerTask(std::string const& name = "ctrl_lib::PotentialFieldsControllerTask", TaskCore::TaskState initial_state = Stopped);
    PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
    ~PotentialFieldsControllerTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

