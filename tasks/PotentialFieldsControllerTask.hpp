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
    virtual bool readSetpoint() = 0;
    /** Read potential field centers. Return false if there are no valid potential centers*/
    virtual bool readFeedback() = 0;
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const base::VectorXd &ctrl_output_raw) = 0;
    /** Write activation function to port*/
    virtual void writeActivationFunction() = 0;

    std::vector<PotentialFieldInfo> field_infos;
    bool has_pot_field_centers, has_feedback;
    base::VectorXd influence_distance;

    void setInfluenceDistance(const base::VectorXd &distance){

        PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

        assert(ctrl != 0);

        // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
        // only do sth. if the size is not zero
        if(distance.size() != 0){

            if(distance.size() != (int)ctrl->fields.size()){
                LOG_ERROR("%s: Max. Influence Distance vector has size %s, but field names size is ",
                          this->getName().c_str(), distance.size(), ctrl->fields.size());
                throw std::invalid_argument("Invalid influence distance");
            }
            for(size_t i = 0; i < ctrl->fields.size(); i++)
                ctrl->fields[i]->influence_distance = distance(i);
        }
    }

public:
    PotentialFieldsControllerTask(std::string const& name = "ctrl_lib::PotentialFieldsControllerTask");
    PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
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

