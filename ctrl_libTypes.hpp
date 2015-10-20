#ifndef ctrl_lib_TYPES_HPP
#define ctrl_lib_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Eigen.hpp>
#include <ctrl_lib/PotentialField.hpp>

namespace ctrl_lib {

enum activationType{NO_ACTIVATION, STEP_ACTIVATION, LINEAR_ACTIVATION, SIGMOID_ACTIVATION};

/** The activation function that is written on a port by the controllers is a sigmoid function:
 *
 *    act = 1 / (1 + exp(-a*(x-c))
 *
 * where
 *    x = input variable
 *    c = offset
 *    a = steepness
 */
struct activationFunction{
    activationFunction() :
        type(NO_ACTIVATION){
    }

    double param_a, param_b;
    activationType type;

    base::VectorXd compute(base::VectorXd& values){
        for(uint i = 0; i < values.size(); i++){
            switch(type){
            case NO_ACTIVATION:
                values(i) = 1;
                break;
            case STEP_ACTIVATION:
                values(i) = 0;
                if(fabs(values(i)) != 0)
                    values(i) = 1;
                else
                    values(i) = 0;
                break;
            case LINEAR_ACTIVATION:
                assert(param_a > 0 && param_a <= 1);
                if(values(i) < param_a)
                    values(i) = (1.0/param_a) * values(i);
                else
                    values(i) = 1;
                break;
            case SIGMOID_ACTIVATION:
                values(i) = 1 / (1 + exp(-param_a*(values(i) - param_b)));
                break;
            default:
                throw std::invalid_argument("Invalid activation type");
            }
        }
        return values;
    }
};

struct PotentialFieldInfo{

    void setFromField(PotentialField* field){
        dimension = field->dimension;
        order = field->order;
        influence_distance = field->influence_distance;
        distance = field->distance;
        gradient = field->gradient;
        pot_field_center = field->pot_field_center;
    }

    /** Dimension of the potential field, e.g. a potential field in 3d space would have size 3.*/
    uint dimension;

    /** Order of the potential field. Defines the relation to the distance of the field. Default will be 1*/
    uint order;

    /** Maximum influence distance of the field. Default will be inf*/
    double influence_distance;

    /** Distance vector to the potential field. */
    base::VectorXd distance;

    /** Gradient vector for this field*/
    base::VectorXd gradient;

    /** Potential field center position*/
    base::VectorXd pot_field_center;
};

}

#endif

