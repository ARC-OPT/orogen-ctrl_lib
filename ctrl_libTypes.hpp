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

struct PotentialFieldInfo{

    void setFromField(PotentialField* field){
        dimension = field->dimension;
        order = field->order;
        influence_distance = field->influence_distance;
        distance = field->distance;
        gradient = field->gradient;
        pot_field_center = field->pot_field_center;
        position = field->position;
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

    /** Current position*/
    base::VectorXd position;
};

}

#endif

