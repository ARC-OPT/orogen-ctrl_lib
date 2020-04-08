/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP
#define CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP

#include "ctrl_lib/CartesianRadialPotentialFieldsBase.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

namespace ctrl_lib {

class PotentialFieldsController;

/*! \class CartesianRadialPotentialFields Implementation of RadialPotentialFields in Cartesian space. Dimension of all fields has to be 3! See ctrl_lib/RadialPotentialField.hpp and
ctrl_lib/PotentialFieldsController.hpp for details  */
class CartesianRadialPotentialFields : public CartesianRadialPotentialFieldsBase
{
    friend class CartesianRadialPotentialFieldsBase;

public:
    CartesianRadialPotentialFields(std::string const& name = "ctrl_lib::CartesianRadialPotentialFields");
    CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianRadialPotentialFields(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();

protected:
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Compute output of the controller*/
    virtual void updateController();
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);

    double influence_distance;
    base::samples::RigidBodyStateSE3 control_output, feedback;
    std::vector<base::samples::RigidBodyState> pot_field_centers;
    PotentialFieldsController* controller;
    std::vector<PotentialFieldInfo> field_infos;
};
}

#endif

