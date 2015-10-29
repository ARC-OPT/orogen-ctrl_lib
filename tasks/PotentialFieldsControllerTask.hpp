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

    virtual void setInfluenceDistance(const base::VectorXd &distance) = 0;

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

