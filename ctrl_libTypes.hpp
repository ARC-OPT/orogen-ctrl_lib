#ifndef ctrl_lib_TYPES_HPP
#define ctrl_lib_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <Eigen/Core>

namespace ctrl_lib {

struct radialPotentialFieldConfig{

   /** Order of the potential field. If M = 0, grad = const. If M = 1, grad ~ 1/d. If M = 2, grad ~ 1/d^2 and so on.*/
   int order;

   /** Maximum influence distance. Beyond that controller will output 0*/
   double dMax; 

   /** Center of the potential field. Size of this vector determines the dimension of the field */
   Eigen::VectorXd x0;
};
}

#endif

