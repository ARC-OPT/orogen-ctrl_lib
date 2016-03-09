/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP
#define CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP

#include "ctrl_lib/CartesianRadialPotentialFieldsBase.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <ctrl_lib/PotentialFieldsController.hpp>

namespace ctrl_lib {

/*! \class CartesianRadialPotentialFields Implementation of RadialPotentialFields in Cartesian space. Dimension of all fields has to be 3! See ctrl_lib/RadialPotentialField.hpp and
ctrl_lib/PotentialFieldsController.hpp for details  */
class CartesianRadialPotentialFields : public CartesianRadialPotentialFieldsBase
{
    friend class CartesianRadialPotentialFieldsBase;
protected:

    base::samples::RigidBodyState control_output, feedback;
    std::vector<base::samples::RigidBodyState> pot_field_centers;
    int order;
    std::vector<PotentialFieldInfo> field_infos;
    bool has_pot_field_centers;

    /** Read actual position. Return false if there is no valid actual position*/
    virtual bool readFeedback();
    /** Read potential field centers. Return false if there are no valid potential centers*/
    virtual bool readSetpoint() ;
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const base::VectorXd &ctrl_output_raw);
    /** Write activation function to port*/
    virtual void writeActivationFunction(){}
    /** Set new potential field centers*/
    void setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers);
    /** Delete all potential fields*/
    void clearPotentialFields();

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

    /** Implementation of reset behavior does not make sense for a Potential Field Controller */
    virtual void reset(){}
};
}

#endif

