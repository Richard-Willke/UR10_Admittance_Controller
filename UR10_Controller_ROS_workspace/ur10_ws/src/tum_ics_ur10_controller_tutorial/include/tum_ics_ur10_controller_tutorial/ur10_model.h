#ifndef UR_ROBOT_LLI_UR10PROPERTY_H
#define UR_ROBOT_LLI_UR10PROPERTY_H

#include <Math/EigenDefs.h>

using namespace Tum;

namespace tum_ics_ur_robot_lli
{

    namespace RobotControllers

    {

        Matrix4d get_ee_forward_kinematics(Vector6d q);
    
        MatrixXd get_ee_jacobian(Vector6d q);

        MatrixXd get_ee_jacobian_dot(Vector6d Q, Vector6d Qp);
        
        MatrixXd get_regressor(Vector6d q, Vector6d dq, Vector6d dqr, Vector6d ddqr);

        MatrixXd get_parameters();

    }

}
#endif 
