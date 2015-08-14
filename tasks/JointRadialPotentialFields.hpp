/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTRADIALPOTENTIALFIELDS_TASK_HPP
#define CTRL_LIB_JOINTRADIALPOTENTIALFIELDS_TASK_HPP

#include "ctrl_lib/JointRadialPotentialFieldsBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/Logging.hpp>

namespace ctrl_lib {

/*! \class JointRadialPotentialFields
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
Implementation of RadialPotentialFields in joint space. Each joint will have its own potential field. See ctrl_lib/RadialPotentialField.hpp and
ctrl_lib/MultiPotentialFields.hpp for details

     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','ctrl_lib::JointRadialPotentialFields')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
class JointRadialPotentialFields : public JointRadialPotentialFieldsBase
{
    friend class JointRadialPotentialFieldsBase;
protected:

    std::vector<std::string> jointNames;

    virtual bool readSetpoints();
    virtual bool readFeedback();
    virtual void writeControlOutput(const Eigen::VectorXd &y);

    base::commands::Joints potFieldCenters;
    base::samples::Joints feedback;
    base::VectorXd maxInfluenceDistance;
    base::commands::Joints controlOutput;
    double potFieldOrder;

    void setMaxInfluenceDistance(const base::VectorXd& distance);
    void setPotFieldCenters(const base::commands::Joints& centers);
    void setOrder(const double order);

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

