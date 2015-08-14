/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP
#define CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP

#include "ctrl_lib/CartesianRadialPotentialFieldsBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace ctrl_lib {

/*! \class CartesianRadialPotentialFields
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
Implementation of RadialPotentialFields in Cartesian space. Dimension of all fields has to be 3! See ctrl_lib/RadialPotentialField.hpp and
ctrl_lib/MultiPotentialFields.hpp for details

     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','ctrl_lib::CartesianRadialPotentialFields')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
class CartesianRadialPotentialFields : public CartesianRadialPotentialFieldsBase
{
    friend class CartesianRadialPotentialFieldsBase;
protected:

    base::samples::RigidBodyState controlOutput, feedback;
    std::vector<base::samples::RigidBodyState> potFieldCenters;
    std::vector<base::VectorXd> gradients;
    base::VectorXd maxInfluenceDistance;
    double potFieldOrder;

    virtual bool readSetpoints();
    virtual bool readFeedback();
    virtual void writeControlOutput(const Eigen::VectorXd &y);

    void setMaxInfluenceDistance(const base::VectorXd& maxInfluenceDistance);
    void setOrder(const double order);
    void setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers);

public:
    CartesianRadialPotentialFields(std::string const& name = "ctrl_lib::CartesianRadialPotentialFields");
    CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianRadialPotentialFields();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

