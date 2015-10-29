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

    base::samples::RigidBodyState control_output, feedback;
    std::vector<base::samples::RigidBodyState> pot_field_centers;
    int order;

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
    /** Set actual position in all potential fields */
    void setActualPosition(const base::samples::RigidBodyState& actual);
    /** Delete all potential fields*/
    void clearPotentialFields();

    virtual void setInfluenceDistance(const base::VectorXd &distance){

        PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

        assert(ctrl != 0);

        // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
        // only do sth. if the size is not zero
        if(distance.size() != 0){

            if(distance.size() != 1){
                LOG_ERROR("%s: Max. Influence Distance has to have size 1, but has size %i",
                          this->getName().c_str(), distance.size());
                throw std::invalid_argument("Invalid influence distance");
            }
            for(size_t i = 0; i < ctrl->fields.size(); i++)
                ctrl->fields[i]->influence_distance = distance(0);
        }
    }

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

    virtual void reset(){}
};
}

#endif

