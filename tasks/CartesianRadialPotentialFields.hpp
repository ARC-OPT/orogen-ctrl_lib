/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP
#define CTRL_LIB_CARTESIANRADIALPOTENTIALFIELDS_TASK_HPP

#include "ctrl_lib/CartesianRadialPotentialFieldsBase.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace ctrl_lib {

/*! \class CartesianRadialPotentialFields Implementation of RadialPotentialFields in Cartesian space. Dimension of all fields has to be 3! See ctrl_lib/RadialPotentialField.hpp and
ctrl_lib/PotentialFieldsController.hpp for details  */
class CartesianRadialPotentialFields : public CartesianRadialPotentialFieldsBase
{
    friend class CartesianRadialPotentialFieldsBase;
protected:

    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw);
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);
    /** Set new potential field centers*/
    void setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers);
    /** Delete all potential fields*/
    void clearPotentialFields();

    double default_influence_distance;
    std::map<std::string, double> influence_distance_map;

    std::map<std::string, double> makeMap(const std::vector<InfluenceDistancePerField>& influence_distance_per_field){
        std::map<std::string, double> tmp_map;
        for(uint i = 0; i < influence_distance_per_field.size(); i++)
            tmp_map[influence_distance_per_field[i].name] = influence_distance_per_field[i].distance;
        return tmp_map;
    }

    base::samples::RigidBodyState control_output, feedback;
    std::vector<base::samples::RigidBodyState> pot_field_centers;
    bool has_pot_fields;

public:
    CartesianRadialPotentialFields(std::string const& name = "ctrl_lib::CartesianRadialPotentialFields");
    CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartesianRadialPotentialFields(){}
    bool configureHook();
    bool startHook(){return CartesianRadialPotentialFieldsBase::startHook();}
    void updateHook(){CartesianRadialPotentialFieldsBase::updateHook();}
    void errorHook(){CartesianRadialPotentialFieldsBase::errorHook();}
    void stopHook(){CartesianRadialPotentialFieldsBase::stopHook();}
    void cleanupHook();

    /** Implementation of reset behavior does not make sense for a Potential Field Controller */
    virtual void reset(){}
};
}

#endif

