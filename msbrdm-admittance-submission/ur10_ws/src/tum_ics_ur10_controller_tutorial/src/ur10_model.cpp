#include <tum_ics_ur10_controller_tutorial/ur10_model.h>
#include <Math/EigenDefs.h>

using namespace Tum;

/* Link lengths were already substituted while calculating the model */




const double l1 = 0.128;
const double l2 = 0.1639;
const double l3 = 0.6127;
const double l4 = 0.0922;
const double l5 = 0.5716;
const double l6 = 0.1;
const double l7 = 0.150;
const double l8 = l3/2;
const double l9 = 0.1157;
const double l10 = l5/2;
const double l11 = 0.1157;
const double l12 = 0.12;
// Value below changes for real robot
const double L_ft = 0.035;


namespace tum_ics_ur_robot_lli
{

    namespace RobotControllers
    {

    

        Eigen::Matrix4d get_ee_forward_kinematics(Vector6d Q){
            double q1=Q(0);
            double q2=Q(1);
            double q3=Q(2);
            double q4=Q(3);
            double q5=Q(4);
            double q6=Q(5);

            // from rohan_print_M_C_G_ur10
            Eigen::MatrixXd T6_0(4,4);
            T6_0 <<   cos(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))), - sin(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))), sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - cos(q5)*sin(q1), l12*sin(q1) - (cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))*(L_ft + l4) - l2*sin(q1) - l11*sin(q1) - l9*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + l3*cos(q1)*cos(q2) + l5*cos(q1)*cos(q2)*cos(q3) - l5*cos(q1)*sin(q2)*sin(q3),
                    - cos(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))),   sin(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))), cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))), l2*cos(q1) - l9*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + l11*cos(q1) - l12*cos(q1) + (L_ft + l4)*(cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) + l3*cos(q2)*sin(q1) + l5*cos(q2)*cos(q3)*sin(q1) - l5*sin(q1)*sin(q2)*sin(q3),
                                                                                                        - sin(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - cos(q5)*cos(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                       cos(q5)*sin(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - cos(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))),                                                  -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                                                 l1 - l3*sin(q2) - l9*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - sin(q5)*(L_ft + l4)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - l5*cos(q2)*sin(q3) - l5*cos(q3)*sin(q2),
                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                            1;




            return T6_0;
        }

        Eigen::MatrixXd get_ee_jacobian(Vector6d Q){
            double q1=Q(0);
            double q2=Q(1);
            double q3=Q(2);
            double q4=Q(3);
            double q5=Q(4);
            double q6=Q(5);

            Eigen::MatrixXd J_ef(6,6);
            J_ef << l12*cos(q1) - l2*cos(q1) - l11*cos(q1) - (L_ft + l4)*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - l3*cos(q2)*sin(q1) + l9*sin(q2 + q3 + q4)*sin(q1) - l5*cos(q2)*cos(q3)*sin(q1) + l5*sin(q1)*sin(q2)*sin(q3),                                                                                  -cos(q1)*(l5*sin(q2 + q3) + l3*sin(q2) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),                                                                                  -cos(q1)*(l5*sin(q2 + q3) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),                                                                                  -cos(q1)*(l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),  (L_ft + l4)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)),                                                   0,
                    l12*sin(q1) - l2*sin(q1) - l11*sin(q1) - (L_ft + l4)*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + l3*cos(q1)*cos(q2) - l9*sin(q2 + q3 + q4)*cos(q1) + l5*cos(q1)*cos(q2)*cos(q3) - l5*cos(q1)*sin(q2)*sin(q3),                                                                                  -sin(q1)*(l5*sin(q2 + q3) + l3*sin(q2) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),                                                                                  -sin(q1)*(l5*sin(q2 + q3) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),                                                                                  -sin(q1)*(l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)), -(L_ft + l4)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)),                                                   0,
                                                                                                                                                                                                                                           0, (L_ft*sin(q2 + q3 + q4 - q5))/2 - (l4*sin(q2 + q3 + q4 + q5))/2 - l5*cos(q2 + q3) - l3*cos(q2) - (L_ft*sin(q2 + q3 + q4 + q5))/2 + (l4*sin(q2 + q3 + q4 - q5))/2 + l9*sin(q2 + q3 + q4), (L_ft*sin(q2 + q3 + q4 - q5))/2 - (l4*sin(q2 + q3 + q4 + q5))/2 - l5*cos(q2 + q3) - (L_ft*sin(q2 + q3 + q4 + q5))/2 + (l4*sin(q2 + q3 + q4 - q5))/2 + l9*sin(q2 + q3 + q4), (L_ft*sin(q2 + q3 + q4 - q5))/2 - (l4*sin(q2 + q3 + q4 + q5))/2 - (L_ft*sin(q2 + q3 + q4 + q5))/2 + (l4*sin(q2 + q3 + q4 - q5))/2 + l9*sin(q2 + q3 + q4), -(L_ft + l4)*(sin(q2 + q3 + q4 + q5)/2 + sin(q2 + q3 + q4 - q5)/2),                                                   0,
                                                                                                                                                                                                                                           0,                                                                                                                                                                                -sin(q1),                                                                                                                                                                   -sin(q1),                                                                                                                                                 -sin(q1),                                         -sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1)*sin(q5) - cos(q5)*sin(q1),
                                                                                                                                                                                                                                           0,                                                                                                                                                                                 cos(q1),                                                                                                                                                                    cos(q1),                                                                                                                                                  cos(q1),                                         -sin(q2 + q3 + q4)*sin(q1), cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5),
                                                                                                                                                                                                                                           1,                                                                                                                                                                                       0,                                                                                                                                                                          0,                                                                                                                                                        0,                                                 -cos(q2 + q3 + q4),                          -sin(q2 + q3 + q4)*sin(q5);
            return J_ef;
        }

        Eigen::MatrixXd get_ee_jacobian_dot(Vector6d Q, Vector6d Qp){
            double q1=Q(0);
            double q2=Q(1);
            double q3=Q(2);
            double q4=Q(3);
            double q5=Q(4);
            double q6=Q(5);

            double qp1=Qp(0);
            double qp2=Qp(1);
            double qp3=Qp(2);
            double qp4=Qp(3);
            double qp5=Qp(4);
            double qp6=Qp(5);

            Eigen::MatrixXd J_ef_dot(6,6);
            J_ef_dot <<  qp3*(l9*cos(q2 + q3 + q4)*sin(q1) + sin(q2 + q3 + q4)*sin(q1)*sin(q5)*(L_ft + l4) + l5*cos(q2)*sin(q1)*sin(q3) + l5*cos(q3)*sin(q1)*sin(q2)) + qp1*((L_ft + l4)*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + l2*sin(q1) + l11*sin(q1) - l12*sin(q1) - l3*cos(q1)*cos(q2) + l9*sin(q2 + q3 + q4)*cos(q1) - l5*cos(q1)*cos(q2)*cos(q3) + l5*cos(q1)*sin(q2)*sin(q3)) + qp4*(l9*cos(q2 + q3 + q4)*sin(q1) + sin(q2 + q3 + q4)*sin(q1)*sin(q5)*(L_ft + l4)) + qp2*sin(q1)*(l3*sin(q2) + l9*cos(q2 + q3 + q4) + l5*cos(q2)*sin(q3) + l5*cos(q3)*sin(q2) + L_ft*sin(q2 + q3 + q4)*sin(q5) + l4*sin(q2 + q3 + q4)*sin(q5)),                                                                             qp4*cos(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp2*cos(q1)*(l5*cos(q2 + q3) + l3*cos(q2) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp1*sin(q1)*(l5*sin(q2 + q3) + l3*sin(q2) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp3*cos(q1)*(l5*cos(q2 + q3) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),                                                                                          qp4*cos(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp2*cos(q1)*(l5*cos(q2 + q3) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp3*cos(q1)*(l5*cos(q2 + q3) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp1*sin(q1)*(l5*sin(q2 + q3) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)), qp2*cos(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp3*cos(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp4*cos(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp1*sin(q1)*(l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)), qp1*(L_ft + l4)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - qp2*sin(q2 + q3 + q4)*cos(q1)*cos(q5)*(L_ft + l4) - qp3*sin(q2 + q3 + q4)*cos(q1)*cos(q5)*(L_ft + l4) - qp4*sin(q2 + q3 + q4)*cos(q1)*cos(q5)*(L_ft + l4),                                                                                                                                                                                   0,
                        - qp4*(l9*cos(q2 + q3 + q4)*cos(q1) + sin(q2 + q3 + q4)*cos(q1)*sin(q5)*(L_ft + l4)) - qp1*((L_ft + l4)*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + l2*cos(q1) + l11*cos(q1) - l12*cos(q1) + l3*cos(q2)*sin(q1) - l9*sin(q2 + q3 + q4)*sin(q1) + l5*cos(q2)*cos(q3)*sin(q1) - l5*sin(q1)*sin(q2)*sin(q3)) - qp3*(l9*cos(q2 + q3 + q4)*cos(q1) + sin(q2 + q3 + q4)*cos(q1)*sin(q5)*(L_ft + l4) + l5*cos(q1)*cos(q2)*sin(q3) + l5*cos(q1)*cos(q3)*sin(q2)) - qp2*cos(q1)*(l3*sin(q2) + l9*cos(q2 + q3 + q4) + l5*cos(q2)*sin(q3) + l5*cos(q3)*sin(q2) + L_ft*sin(q2 + q3 + q4)*sin(q5) + l4*sin(q2 + q3 + q4)*sin(q5)),                                                                             qp4*sin(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp2*sin(q1)*(l5*cos(q2 + q3) + l3*cos(q2) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp1*cos(q1)*(l5*sin(q2 + q3) + l3*sin(q2) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp3*sin(q1)*(l5*cos(q2 + q3) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)),                                                                                          qp4*sin(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp1*cos(q1)*(l5*sin(q2 + q3) + l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp2*sin(q1)*(l5*cos(q2 + q3) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp3*sin(q1)*(l5*cos(q2 + q3) - l9*sin(q2 + q3 + q4) + cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)), qp2*sin(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) - qp1*cos(q1)*(l9*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp3*sin(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)) + qp4*sin(q1)*(l9*sin(q2 + q3 + q4) - cos(q2 + q3 + q4)*sin(q5)*(L_ft + l4)), qp1*(L_ft + l4)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - qp2*sin(q2 + q3 + q4)*cos(q5)*sin(q1)*(L_ft + l4) - qp3*sin(q2 + q3 + q4)*cos(q5)*sin(q1)*(L_ft + l4) - qp4*sin(q2 + q3 + q4)*cos(q5)*sin(q1)*(L_ft + l4),                                                                                                                                                                                  0,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0, qp2*(l5*sin(q2 + q3) - (l4*cos(q2 + q3 + q4 + q5))/2 - (L_ft*cos(q2 + q3 + q4 + q5))/2 + (L_ft*cos(q2 + q3 + q4 - q5))/2 + l3*sin(q2) + (l4*cos(q2 + q3 + q4 - q5))/2 + l9*cos(q2 + q3 + q4)) + qp3*(l5*sin(q2 + q3) - (l4*cos(q2 + q3 + q4 + q5))/2 - (L_ft*cos(q2 + q3 + q4 + q5))/2 + (L_ft*cos(q2 + q3 + q4 - q5))/2 + (l4*cos(q2 + q3 + q4 - q5))/2 + l9*cos(q2 + q3 + q4)) + qp4*(l9*cos(q2 + q3 + q4) + L_ft*sin(q2 + q3 + q4)*sin(q5) + l4*sin(q2 + q3 + q4)*sin(q5)), qp2*(l5*sin(q2 + q3) - (l4*cos(q2 + q3 + q4 + q5))/2 - (L_ft*cos(q2 + q3 + q4 + q5))/2 + (L_ft*cos(q2 + q3 + q4 - q5))/2 + (l4*cos(q2 + q3 + q4 - q5))/2 + l9*cos(q2 + q3 + q4)) + qp3*(l5*sin(q2 + q3) - (l4*cos(q2 + q3 + q4 + q5))/2 - (L_ft*cos(q2 + q3 + q4 + q5))/2 + (L_ft*cos(q2 + q3 + q4 - q5))/2 + (l4*cos(q2 + q3 + q4 - q5))/2 + l9*cos(q2 + q3 + q4)) + qp4*(l9*cos(q2 + q3 + q4) + L_ft*sin(q2 + q3 + q4)*sin(q5) + l4*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                         (qp2 + qp3 + qp4)*(l9*cos(q2 + q3 + q4) + L_ft*sin(q2 + q3 + q4)*sin(q5) + l4*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                              -((L_ft + l4)*(cos(q2 + q3 + q4 + q5) + cos(q2 + q3 + q4 - q5))*(qp2 + qp3 + qp4))/2,                                                                                                                                                                                   0,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -qp1*cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                     -qp1*cos(q1),                                                                                                                                                                                                                                                                                                      -qp1*cos(q1),                 qp1*sin(q2 + q3 + q4)*sin(q1) - (qp3*(cos(q1 + q2 + q3 + q4) + cos(q2 - q1 + q3 + q4)))/2 - (qp4*(cos(q1 + q2 + q3 + q4) + cos(q2 - q1 + q3 + q4)))/2 - (qp2*(cos(q1 + q2 + q3 + q4) + cos(q2 - q1 + q3 + q4)))/2, - qp1*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - qp2*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - qp3*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - qp4*sin(q2 + q3 + q4)*cos(q1)*sin(q5),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -qp1*sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                                                     -qp1*sin(q1),                                                                                                                                                                                                                                                                                                      -qp1*sin(q1),                                                                                                   - qp1*sin(q2 + q3 + q4)*cos(q1) - qp2*cos(q2 + q3 + q4)*sin(q1) - qp3*cos(q2 + q3 + q4)*sin(q1) - qp4*cos(q2 + q3 + q4)*sin(q1), qp1*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - qp1*cos(q5)*sin(q1) - qp2*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - qp3*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - qp4*sin(q2 + q3 + q4)*sin(q1)*sin(q5),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                               sin(q2 + q3 + q4)*(qp2 + qp3 + qp4),                                                                                                                                        -cos(q2 + q3 + q4)*sin(q5)*(qp2 + qp3 + qp4);




            
            return J_ef_dot;            
        }

        Eigen::MatrixXd get_regressor(Vector6d Q, Vector6d Qp,Vector6d Qr_dot, Vector6d Qr_ddot){


            //Joint Errors
            //DeltaQ = Q - Qd;
            //DeltaQp = Qp - Qdp;
            double q1=Q(0);
            double q2=Q(1);
            double q3=Q(2);
            double q4=Q(3);
            double q5=Q(4);
            double q6=Q(5);

            double qp1=Qp(0);
            double qp2=Qp(1);
            double qp3=Qp(2);
            double qp4=Qp(3);
            double qp5=Qp(4);
            double qp6=Qp(5);

            //Eigen::MatrixXd Qr_dot;
            //Qr_dot << Qdp - Kp * DeltaQ;

            double qpr1 = Qr_dot(0);
            double qpr2 = Qr_dot(1);
            double qpr3 = Qr_dot(2);
            double qpr4 = Qr_dot(3);
            double qpr5 = Qr_dot(4);
            double qpr6 = Qr_dot(5);

            double qppr1 = Qr_ddot(0);
            double qppr2 = Qr_ddot(1);
            double qppr3 = Qr_ddot(2);
            double qppr4 = Qr_ddot(3);
            double qppr5 = Qr_ddot(4);
            double qppr6 = Qr_ddot(5);     


            // real size is (6,170), but this way i dont have to redo all the indices if i remap it in the end.
            Eigen::MatrixXd Yr(7,164);  
            Yr.setZero();
            Yr( 1,1) =qppr1;



            Yr( 1,2) =qppr1/2 - (qppr1*cos(2*q2))/2 + (qp1*qpr2*sin(2*q2))/2 + (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,3) =qppr1*sin(2*q2) + qp1*qpr2*cos(2*q2) + qp2*qpr1*cos(2*q2);



            Yr( 1,4) =- qppr2*sin(q2) - qp2*qpr2*cos(q2);



            Yr( 1,5) =qppr1/2 + (qppr1*cos(2*q2))/2 - (qp1*qpr2*sin(2*q2))/2 - (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,6) =qp2*qpr2*sin(q2) - qppr2*cos(q2);



            Yr( 1,7) =0;



            Yr( 1,8) =qppr1/2 - (qppr1*cos(2*q2 + 2*q3))/2 + (qp1*qpr2*sin(2*q2 + 2*q3))/2 + (qp2*qpr1*sin(2*q2 + 2*q3))/2 + (qp1*qpr3*sin(2*q2 + 2*q3))/2 + (qp3*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 1,9) =qppr1*sin(2*q2 + 2*q3) + qp1*qpr2*cos(2*q2 + 2*q3) + qp2*qpr1*cos(2*q2 + 2*q3) + qp1*qpr3*cos(2*q2 + 2*q3) + qp3*qpr1*cos(2*q2 + 2*q3);



            Yr( 1,10) =- qppr2*sin(q2 + q3) - qppr3*sin(q2 + q3) - qp2*qpr2*cos(q2 + q3) - qp2*qpr3*cos(q2 + q3) - qp3*qpr2*cos(q2 + q3) - qp3*qpr3*cos(q2 + q3);



            Yr( 1,11) =qppr1/2 + (qppr1*cos(2*q2 + 2*q3))/2 - (qp1*qpr2*sin(2*q2 + 2*q3))/2 - (qp2*qpr1*sin(2*q2 + 2*q3))/2 - (qp1*qpr3*sin(2*q2 + 2*q3))/2 - (qp3*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 1,12) =qp2*qpr2*sin(q2 + q3) - qppr3*cos(q2 + q3) - qppr2*cos(q2 + q3) + qp2*qpr3*sin(q2 + q3) + qp3*qpr2*sin(q2 + q3) + qp3*qpr3*sin(q2 + q3);



            Yr( 1,13) =0;



            Yr( 1,14) =qppr1/2 + qppr2/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 - (qppr2*cos(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 + qp2*qpr2*sin(2*q2 + 2*q3 + 2*q4) + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp3*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp4*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 1,15) =qppr1*sin(2*q2 + 2*q3 + 2*q4) + qppr2*sin(2*q2 + 2*q3 + 2*q4) + qp1*qpr2*cos(2*q2 + 2*q3 + 2*q4) + qp2*qpr1*cos(2*q2 + 2*q3 + 2*q4) + qp1*qpr3*cos(2*q2 + 2*q3 + 2*q4) + 2*qp2*qpr2*cos(2*q2 + 2*q3 + 2*q4) + qp3*qpr1*cos(2*q2 + 2*q3 + 2*q4) + qp1*qpr4*cos(2*q2 + 2*q3 + 2*q4) + qp2*qpr3*cos(2*q2 + 2*q3 + 2*q4) + qp3*qpr2*cos(2*q2 + 2*q3 + 2*q4) + qp4*qpr1*cos(2*q2 + 2*q3 + 2*q4) + qp2*qpr4*cos(2*q2 + 2*q3 + 2*q4) + qp4*qpr2*cos(2*q2 + 2*q3 + 2*q4);



            Yr( 1,16) =- qppr3*sin(q2 + q3 + q4) - qppr4*sin(q2 + q3 + q4) - (qp2*qpr3*cos(q2 + q3 + q4))/2 - (qp3*qpr2*cos(q2 + q3 + q4))/2 - (qp2*qpr4*cos(q2 + q3 + q4))/2 - qp3*qpr3*cos(q2 + q3 + q4) - (qp4*qpr2*cos(q2 + q3 + q4))/2 - qp3*qpr4*cos(q2 + q3 + q4) - qp4*qpr3*cos(q2 + q3 + q4) - qp4*qpr4*cos(q2 + q3 + q4);



            Yr( 1,17) =qppr1/2 + qppr2/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 + (qppr2*cos(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 - qp2*qpr2*sin(2*q2 + 2*q3 + 2*q4) - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 1,18) =(qp2*qpr3*sin(q2 + q3 + q4))/2 - qppr4*cos(q2 + q3 + q4) - qppr3*cos(q2 + q3 + q4) + (qp3*qpr2*sin(q2 + q3 + q4))/2 + (qp2*qpr4*sin(q2 + q3 + q4))/2 + qp3*qpr3*sin(q2 + q3 + q4) + (qp4*qpr2*sin(q2 + q3 + q4))/2 + qp3*qpr4*sin(q2 + q3 + q4) + qp4*qpr3*sin(q2 + q3 + q4) + qp4*qpr4*sin(q2 + q3 + q4);



            Yr( 1,19) =0;



            Yr( 1,20) =qppr1/4 + (qppr1*cos(2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4))/4 + (qppr2*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr2*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr3*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr3*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr4*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr4*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(2*q5))/4 - (qp5*qpr1*sin(2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 1,21) =(qppr2*sin(q2 + q3 + q4 - 2*q5))/2 - (qppr1*sin(2*q5))/2 + (qppr2*sin(q2 + q3 + q4 + 2*q5))/2 + (qppr3*sin(q2 + q3 + q4 - 2*q5))/2 + (qppr3*sin(q2 + q3 + q4 + 2*q5))/2 + (qppr4*sin(q2 + q3 + q4 - 2*q5))/2 + (qppr4*sin(q2 + q3 + q4 + 2*q5))/2 - (qppr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qppr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp2*qpr2*cos(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr2*cos(q2 + q3 + q4 + 2*q5))/2 + (qp2*qpr3*cos(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr3*cos(q2 + q3 + q4 + 2*q5))/2 + (qp3*qpr2*cos(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr2*cos(q2 + q3 + q4 + 2*q5))/2 + (qp2*qpr4*cos(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr4*cos(q2 + q3 + q4 + 2*q5))/2 + (qp3*qpr3*cos(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr3*cos(q2 + q3 + q4 + 2*q5))/2 + (qp4*qpr2*cos(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr2*cos(q2 + q3 + q4 + 2*q5))/2 - (qp2*qpr5*cos(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr5*cos(q2 + q3 + q4 + 2*q5))/2 + (qp3*qpr4*cos(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr4*cos(q2 + q3 + q4 + 2*q5))/2 + (qp4*qpr3*cos(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr3*cos(q2 + q3 + q4 + 2*q5))/2 - (qp5*qpr2*cos(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr2*cos(q2 + q3 + q4 + 2*q5))/2 - (qp3*qpr5*cos(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr5*cos(q2 + q3 + q4 + 2*q5))/2 + (qp4*qpr4*cos(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr4*cos(q2 + q3 + q4 + 2*q5))/2 - (qp5*qpr3*cos(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr3*cos(q2 + q3 + q4 + 2*q5))/2 - (qp4*qpr5*cos(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr5*cos(q2 + q3 + q4 + 2*q5))/2 - (qp5*qpr4*cos(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr4*cos(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr2*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr2*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp2*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp2*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp1*qpr3*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr3*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp3*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp3*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp1*qpr4*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr4*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp4*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp4*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp1*qpr5*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr5*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp1*qpr5*cos(2*q5))/2 - (qp5*qpr1*cos(2*q5))/2;



            Yr( 1,22) =(qppr5*sin(q2 + q3 + q4 + q5))/2 - (qppr3*sin(q2 + q3 + q4 + q5))/2 - (qppr4*sin(q2 + q3 + q4 + q5))/2 - (qppr2*sin(q2 + q3 + q4 + q5))/2 - (qppr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qppr2*sin(q2 + q3 + q4 - q5))/2 + (qppr3*sin(q2 + q3 + q4 - q5))/2 + (qppr4*sin(q2 + q3 + q4 - q5))/2 + (qppr5*sin(q2 + q3 + q4 - q5))/2 - (qppr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp2*qpr2*cos(q2 + q3 + q4 - q5))/2 + (qp2*qpr3*cos(q2 + q3 + q4 - q5))/2 + (qp3*qpr2*cos(q2 + q3 + q4 - q5))/2 + (qp2*qpr4*cos(q2 + q3 + q4 - q5))/2 + (qp3*qpr3*cos(q2 + q3 + q4 - q5))/2 + (qp4*qpr2*cos(q2 + q3 + q4 - q5))/2 + (qp3*qpr4*cos(q2 + q3 + q4 - q5))/2 + (qp4*qpr3*cos(q2 + q3 + q4 - q5))/2 + (qp4*qpr4*cos(q2 + q3 + q4 - q5))/2 - (qp5*qpr5*cos(q2 + q3 + q4 - q5))/2 - (qp1*qpr2*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp2*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr3*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp3*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr4*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp4*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr5*cos(2*q2 + 2*q3 + 2*q4 - q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/4 - (qp2*qpr2*cos(q2 + q3 + q4 + q5))/2 - (qp2*qpr3*cos(q2 + q3 + q4 + q5))/2 - (qp3*qpr2*cos(q2 + q3 + q4 + q5))/2 - (qp2*qpr4*cos(q2 + q3 + q4 + q5))/2 - (qp3*qpr3*cos(q2 + q3 + q4 + q5))/2 - (qp4*qpr2*cos(q2 + q3 + q4 + q5))/2 - (qp3*qpr4*cos(q2 + q3 + q4 + q5))/2 - (qp4*qpr3*cos(q2 + q3 + q4 + q5))/2 - (qp4*qpr4*cos(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*cos(q2 + q3 + q4 + q5))/2 - (qp1*qpr2*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp2*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr3*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp3*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr4*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp4*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr5*cos(2*q2 + 2*q3 + 2*q4 + q5))/4 - (qp5*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 1,23) =qppr1/4 - (qppr1*cos(2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4))/4 - (qppr2*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr2*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr3*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr3*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr4*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr4*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr5*sin(2*q5))/4 + (qp5*qpr1*sin(2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 1,24) =(qppr5*cos(q2 + q3 + q4 + q5))/2 - (qppr3*cos(q2 + q3 + q4 + q5))/2 - (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 + q5))/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 - q5))/2 - (qppr3*cos(q2 + q3 + q4 - q5))/2 - (qppr4*cos(q2 + q3 + q4 - q5))/2 - (qppr5*cos(q2 + q3 + q4 - q5))/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 + (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5))/4 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 1,25) =qppr1/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 - qppr5*cos(q2 + q3 + q4) + (qp2*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr2*sin(q2 + q3 + q4))/2 + (qp3*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr3*sin(q2 + q3 + q4))/2 + (qp4*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr4*sin(q2 + q3 + q4))/2 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 1,26) =(3*qppr1)/8 + (qppr1*cos(2*q5))/8 - (qppr1*cos(2*q6))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/8 + (qppr2*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (qppr2*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (qppr3*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (qppr3*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (qppr4*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (qppr4*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (qppr5*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (qppr5*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (qppr1*cos(2*q5 - 2*q6))/16 + (qppr1*cos(2*q5 + 2*q6))/16 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qppr2*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (qppr2*cos(q2 + q3 + q4 - q5 + 2*q6))/8 + (qppr2*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qppr2*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qppr2*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qppr2*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qppr3*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (qppr3*cos(q2 + q3 + q4 - q5 + 2*q6))/8 + (qppr3*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qppr3*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qppr3*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qppr3*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qppr4*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (qppr4*cos(q2 + q3 + q4 - q5 + 2*q6))/8 + (qppr4*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qppr4*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qppr4*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qppr4*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qppr5*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (qppr5*cos(q2 + q3 + q4 - q5 + 2*q6))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qppr2*cos(q2 + q3 + q4 - 2*q5))/8 - (qppr2*cos(q2 + q3 + q4 + 2*q5))/8 + (qppr3*cos(q2 + q3 + q4 - 2*q5))/8 - (qppr3*cos(q2 + q3 + q4 + 2*q5))/8 + (qppr4*cos(q2 + q3 + q4 - 2*q5))/8 - (qppr4*cos(q2 + q3 + q4 + 2*q5))/8 + (qppr5*cos(q2 + q3 + q4 - 2*q6))/4 + (qppr5*cos(q2 + q3 + q4 + 2*q6))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (qppr5*cos(q2 + q3 + q4))/2 + (qp2*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp2*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp2*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp2*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp3*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp3*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp2*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp2*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp3*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp3*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp4*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp4*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp3*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp3*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp4*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp4*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp2*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp2*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp4*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp4*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp6*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp6*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp2*qpr6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp2*qpr6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp2*qpr6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp2*qpr6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp3*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp3*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp6*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp6*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp6*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp6*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp6*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp6*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp3*qpr6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp3*qpr6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp3*qpr6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp3*qpr6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp4*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp4*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp5*qpr5*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp5*qpr5*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp6*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp6*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp6*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp6*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp6*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp6*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp4*qpr6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp4*qpr6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp4*qpr6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp4*qpr6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp5*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp5*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp6*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp6*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp6*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp6*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp6*qpr5*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp6*qpr5*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qpr5*sin(q2 + q3 + q4 - 2*q6))/8 - (qp2*qpr5*sin(q2 + q3 + q4 + 2*q6))/8 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp5*qpr2*sin(q2 + q3 + q4 - 2*q6))/8 - (qp5*qpr2*sin(q2 + q3 + q4 + 2*q6))/8 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr5*sin(q2 + q3 + q4 - 2*q6))/8 - (qp3*qpr5*sin(q2 + q3 + q4 + 2*q6))/8 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/8 - (qp5*qpr3*sin(q2 + q3 + q4 - 2*q6))/8 - (qp5*qpr3*sin(q2 + q3 + q4 + 2*q6))/8 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr5*sin(q2 + q3 + q4 - 2*q6))/8 - (qp4*qpr5*sin(q2 + q3 + q4 + 2*q6))/8 - (qp5*qpr4*sin(q2 + q3 + q4 - 2*q6))/8 - (qp5*qpr4*sin(q2 + q3 + q4 + 2*q6))/8 + (qp5*qpr6*sin(q2 + q3 + q4 - 2*q6))/4 - (qp5*qpr6*sin(q2 + q3 + q4 + 2*q6))/4 + (qp6*qpr5*sin(q2 + q3 + q4 - 2*q6))/4 - (qp6*qpr5*sin(q2 + q3 + q4 + 2*q6))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (3*qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp2*qpr5*sin(q2 + q3 + q4))/4 + (qp5*qpr2*sin(q2 + q3 + q4))/4 + (qp3*qpr5*sin(q2 + q3 + q4))/4 + (qp5*qpr3*sin(q2 + q3 + q4))/4 + (qp4*qpr5*sin(q2 + q3 + q4))/4 + (qp5*qpr4*sin(q2 + q3 + q4))/4 - (qp1*qpr5*sin(2*q5))/8 - (qp5*qpr1*sin(2*q5))/8 + (qp1*qpr6*sin(2*q6))/8 + (qp6*qpr1*sin(2*q6))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp2*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp2*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp2*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp2*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp3*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp3*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp2*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp2*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp3*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp3*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp4*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp4*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp3*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp3*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp4*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp4*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp2*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp2*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp4*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp4*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp6*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp6*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp3*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp3*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp6*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp6*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp4*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp4*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp5*qpr5*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp5*qpr5*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp6*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp6*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp5*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp5*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp6*qpr5*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp6*qpr5*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp1*qpr5*sin(2*q5 - 2*q6))/16 - (qp1*qpr5*sin(2*q5 + 2*q6))/16 - (qp5*qpr1*sin(2*q5 - 2*q6))/16 - (qp5*qpr1*sin(2*q5 + 2*q6))/16 + (qp1*qpr6*sin(2*q5 - 2*q6))/16 - (qp1*qpr6*sin(2*q5 + 2*q6))/16 + (qp6*qpr1*sin(2*q5 - 2*q6))/16 - (qp6*qpr1*sin(2*q5 + 2*q6))/16 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8;



            Yr( 1,27) =(qppr1*sin(2*q6))/4 - (qppr1*cos(2*q5)*sin(2*q6))/4 + (qp1*qpr6*cos(2*q6))/4 + (qp6*qpr1*cos(2*q6))/4 - (qp1*qpr6*cos(2*q5)*cos(2*q6))/4 - (qp6*qpr1*cos(2*q5)*cos(2*q6))/4 + (qp1*qpr5*sin(2*q5)*sin(2*q6))/4 + (qp5*qpr1*sin(2*q5)*sin(2*q6))/4 - qppr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4) + (3*qppr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6))/4 + qppr5*sin(2*q6)*cos(q2)*sin(q3)*sin(q4) + qppr5*sin(2*q6)*cos(q3)*sin(q2)*sin(q4) + qppr5*sin(2*q6)*cos(q4)*sin(q2)*sin(q3) - (3*qppr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qppr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6))/4 - (3*qppr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6))/4 - (qppr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 - (qppr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(2*q6))/4 - (qppr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q6))/4 + (qppr2*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qppr3*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qppr4*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (3*qp1*qpr6*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6))/4 + (3*qp6*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp2*qpr5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp2*qpr5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp5*qpr2*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp5*qpr2*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp5*qpr2*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp3*qpr5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp3*qpr5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp3*qpr5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp5*qpr3*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp5*qpr3*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp5*qpr3*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp4*qpr5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp4*qpr5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp4*qpr5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp5*qpr4*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp5*qpr4*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp5*qpr4*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + qp5*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4) + qp5*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4) + qp5*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3) + qp6*qpr5*cos(2*q6)*cos(q2)*sin(q3)*sin(q4) + qp6*qpr5*cos(2*q6)*cos(q3)*sin(q2)*sin(q4) + qp6*qpr5*cos(2*q6)*cos(q4)*sin(q2)*sin(q3) - (3*qp1*qpr2*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qp1*qpr2*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 - (3*qp1*qpr2*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 - (3*qp2*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qp2*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 - (3*qp2*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 - (3*qp1*qpr3*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qp1*qpr3*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 - (3*qp1*qpr3*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 - (3*qp3*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qp3*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 - (3*qp3*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 - (3*qp1*qpr4*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qp1*qpr4*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 - (3*qp1*qpr4*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 - (3*qp4*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 - (3*qp4*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 - (3*qp4*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 - (3*qp1*qpr6*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4))/4 - (3*qp1*qpr6*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4))/4 - (3*qp1*qpr6*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3))/4 - (3*qp6*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4))/4 - (3*qp6*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4))/4 - (3*qp6*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3))/4 - (qp2*qpr5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp5*qpr2*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp5*qpr3*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp5*qpr4*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (3*qp1*qpr2*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp2*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp1*qpr3*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp3*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp1*qpr4*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp4*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qppr2*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qppr3*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qppr4*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qppr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - qppr5*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - qppr5*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + qppr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*cos(q5) + qppr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*cos(q5) + qppr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*cos(q5) - qppr2*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qppr2*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qppr2*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qppr3*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qppr3*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qppr3*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qppr4*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qppr4*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qppr4*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qppr5*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - qppr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5) + (qppr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q6))/4 - (qppr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qppr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qppr2*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qppr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qppr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qppr3*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qppr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qppr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qppr4*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - qp5*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4) - qp6*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q4) - qp1*qpr2*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) - qp1*qpr2*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) - qp1*qpr2*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) - qp2*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) - qp2*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) - qp2*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) - qp1*qpr3*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) - qp1*qpr3*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) - qp1*qpr3*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) - qp3*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) - qp3*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) - qp3*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) - qp1*qpr4*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) - qp1*qpr4*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) - qp1*qpr4*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) - qp4*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) - qp4*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) - qp4*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) - (qp1*qpr5*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*sin(q5))/2 - (qp1*qpr5*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*sin(q5))/2 - (qp1*qpr5*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(q5))/2 - (qp5*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*sin(q5))/2 - (qp5*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*sin(q5))/2 - (qp5*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(q5))/2 - qp1*qpr6*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5) - qp1*qpr6*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6)*cos(q5) - qp1*qpr6*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6)*cos(q5) - qp6*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5) - qp6*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6)*cos(q5) - qp6*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6)*cos(q5) + qp2*qpr2*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp2*qpr3*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp3*qpr2*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp2*qpr4*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp3*qpr3*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp4*qpr2*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp3*qpr4*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp4*qpr3*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp2*qpr6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp2*qpr6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp2*qpr6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp4*qpr4*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr2*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr2*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp6*qpr2*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp3*qpr6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp3*qpr6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp3*qpr6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp6*qpr3*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr3*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp6*qpr3*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp4*qpr6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp4*qpr6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp4*qpr6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp5*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr4*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr4*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp6*qpr4*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp5*qpr6*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - qp6*qpr5*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + (qp1*qpr5*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5))/2 + (qp5*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5))/2 + qp1*qpr6*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5) + qp6*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5) + (qp1*qpr6*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6))/4 + (qp6*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6))/4 - (qp2*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp2*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp3*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp2*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp3*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp4*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp2*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp2*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp2*qpr5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp3*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp4*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp5*qpr2*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr2*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr2*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp2*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp2*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp2*qpr6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp3*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp3*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp3*qpr5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp4*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp5*qpr3*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr3*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr3*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp6*qpr2*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp6*qpr2*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp6*qpr2*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp3*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp3*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp3*qpr6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp4*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp4*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp4*qpr5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr4*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr4*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr4*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp6*qpr3*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp6*qpr3*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp6*qpr3*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp4*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp4*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp4*qpr6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp6*qpr4*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp6*qpr4*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp6*qpr4*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp1*qpr2*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 - (qp1*qpr2*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 - (qp1*qpr2*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 - (qp2*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 - (qp2*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 - (qp2*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 - (qp1*qpr3*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 - (qp1*qpr3*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 - (qp1*qpr3*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 - (qp3*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 - (qp3*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 - (qp3*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 - (qp1*qpr4*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 - (qp1*qpr4*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 - (qp1*qpr4*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 - (qp4*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 - (qp4*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 - (qp4*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 - (qp1*qpr5*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(2*q6))/4 - (qp5*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(2*q6))/4 - (qp1*qpr6*cos(2*q2)*cos(2*q5)*cos(2*q6)*sin(2*q3)*sin(2*q4))/4 - (qp1*qpr6*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q4))/4 - (qp1*qpr6*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3))/4 - (qp6*qpr1*cos(2*q2)*cos(2*q5)*cos(2*q6)*sin(2*q3)*sin(2*q4))/4 - (qp6*qpr1*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q4))/4 - (qp6*qpr1*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3))/4 + (qp2*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp2*qpr2*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp2*qpr2*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp2*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp2*qpr3*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp2*qpr3*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp3*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr2*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp3*qpr2*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp2*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp2*qpr4*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp2*qpr4*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp3*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr3*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp3*qpr3*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp4*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr2*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp4*qpr2*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp2*qpr5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr4*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp3*qpr4*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp4*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr3*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp4*qpr3*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp5*qpr2*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp2*qpr6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr4*sin(2*q5)*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp4*qpr4*sin(2*q5)*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp5*qpr3*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr2*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr4*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr3*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr4*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr2*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (qp2*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (qp1*qpr3*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (qp3*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (qp1*qpr4*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (qp4*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + (qp1*qpr5*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(2*q6))/4 + (qp1*qpr5*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(2*q6))/4 + (qp1*qpr5*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(2*q6))/4 + (qp5*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(2*q6))/4 + (qp5*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(2*q6))/4 + (qp5*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(2*q6))/4 + qp1*qpr2*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) + qp2*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) + qp1*qpr3*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) + qp3*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) + qp1*qpr4*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) + qp4*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) - qp2*qpr2*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp2*qpr2*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp2*qpr2*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp2*qpr3*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp2*qpr3*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp2*qpr3*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp3*qpr2*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp3*qpr2*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp3*qpr2*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp2*qpr4*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp2*qpr4*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp2*qpr4*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp3*qpr3*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp3*qpr3*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp3*qpr3*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp4*qpr2*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp4*qpr2*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp4*qpr2*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp3*qpr4*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp3*qpr4*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp3*qpr4*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp4*qpr3*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp4*qpr3*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp4*qpr3*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp2*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp4*qpr4*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) - qp4*qpr4*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) - qp4*qpr4*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp6*qpr2*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp3*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp6*qpr3*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp4*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp5*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4)*sin(q5) + qp5*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3)*sin(q5) + qp5*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - qp6*qpr4*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp5*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) + qp5*qpr6*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + qp5*qpr6*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + qp6*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) + qp6*qpr5*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + qp6*qpr5*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2);



            Yr( 1,28) =(qppr1*sin(2*q5)*cos(q6))/2 + (qp1*qpr5*cos(2*q5)*cos(q6))/2 + (qp5*qpr1*cos(2*q5)*cos(q6))/2 - (qp1*qpr6*sin(2*q5)*sin(q6))/4 - (qp6*qpr1*sin(2*q5)*sin(q6))/4 - qppr6*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qppr6*cos(q2)*sin(q3)*sin(q4)*sin(q6) + qppr6*cos(q3)*sin(q2)*sin(q4)*sin(q6) + qppr6*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qppr2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - qppr3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - qppr4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - qppr6*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) - qppr6*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - qppr6*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qppr2*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + qppr2*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) + qppr2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + qppr3*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + qppr3*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) + qppr3*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + qppr4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + qppr4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) + qppr4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qppr5*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) - qppr5*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) - qppr5*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) + qppr6*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qppr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5)*sin(q6) + qppr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5)*sin(q6) + qppr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5)*sin(q6) + qppr5*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qppr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qppr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qppr2*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qppr2*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qppr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qppr3*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qppr3*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qppr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qppr4*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qppr4*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - (qppr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*cos(q6))/2 + qppr2*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qppr3*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qppr4*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp6*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q6) + (qppr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qppr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qppr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp2*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp2*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp2*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr2*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr2*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr2*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp3*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp3*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp3*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr3*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr3*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr3*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp4*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp4*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp4*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr4*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr4*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr4*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + qp6*qpr6*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp6*qpr6*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp6*qpr6*cos(q4)*cos(q6)*sin(q2)*sin(q3) - (qp2*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr2*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp3*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr3*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp4*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr4*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr2*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qp1*qpr2*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp1*qpr2*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 + (qp2*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qp2*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp2*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 + (qp1*qpr3*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qp1*qpr3*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp1*qpr3*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 + (qp3*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qp3*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp3*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 + (qp1*qpr4*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qp1*qpr4*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp1*qpr4*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 + (qp4*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + (qp4*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 + (qp4*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 + (qp1*qpr5*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*cos(q6))/2 + (qp1*qpr5*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*cos(q6))/2 + (qp1*qpr5*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*cos(q6))/2 + (qp5*qpr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*cos(q6))/2 + (qp5*qpr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*cos(q6))/2 + (qp5*qpr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*cos(q6))/2 + (qp1*qpr6*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(q6))/4 + (qp6*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(q6))/4 - qp2*qpr5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp5*qpr2*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp2*qpr6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - qp3*qpr5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp5*qpr3*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp6*qpr2*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp3*qpr6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - qp4*qpr5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp5*qpr4*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp6*qpr3*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp4*qpr6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr4*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr2*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp2*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp1*qpr3*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp3*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp1*qpr4*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp4*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp1*qpr6*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/4 - (qp1*qpr6*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(q6))/4 - (qp1*qpr6*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(q6))/4 - (qp6*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/4 - (qp6*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(q6))/4 - (qp6*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(q6))/4 - qp2*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qp6*qpr2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qp3*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qp6*qpr3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qp4*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qp6*qpr4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + qp2*qpr2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp2*qpr2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp2*qpr2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp2*qpr3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp2*qpr3*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp2*qpr3*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp3*qpr2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp3*qpr2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp3*qpr2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp2*qpr4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp2*qpr4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp2*qpr4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp3*qpr3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp3*qpr3*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp3*qpr3*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp4*qpr2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp4*qpr2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp4*qpr2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp3*qpr4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp3*qpr4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp3*qpr4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp4*qpr3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp4*qpr3*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp4*qpr3*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp2*qpr6*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qp2*qpr6*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qp2*qpr6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qp4*qpr4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp4*qpr4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp4*qpr4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp6*qpr2*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qp6*qpr2*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qp6*qpr2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qp3*qpr6*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qp3*qpr6*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qp3*qpr6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qp6*qpr3*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qp6*qpr3*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qp6*qpr3*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qp4*qpr6*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qp4*qpr6*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qp4*qpr6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - qp5*qpr5*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) - qp5*qpr5*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) - qp5*qpr5*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp6*qpr4*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qp6*qpr4*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qp6*qpr4*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qp6*qpr6*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qp6*qpr6*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qp6*qpr6*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qp1*qpr2*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) + qp2*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) + qp1*qpr3*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) + qp3*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) + qp1*qpr4*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) + qp4*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) + (qp1*qpr5*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/2 + (qp1*qpr5*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*sin(q6))/2 + (qp1*qpr5*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*sin(q6))/2 + (qp5*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/2 + (qp5*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*sin(q6))/2 + (qp5*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*sin(q6))/2 + (qp1*qpr6*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/2 + (qp1*qpr6*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q6)*sin(q5))/2 + (qp1*qpr6*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q6)*sin(q5))/2 + (qp6*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/2 + (qp6*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q6)*sin(q5))/2 + (qp6*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q6)*sin(q5))/2 - qp2*qpr2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr5*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp6*qpr6*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp1*qpr2*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qp1*qpr2*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) - qp1*qpr2*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) - qp2*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qp2*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) - qp2*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) - qp1*qpr3*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qp1*qpr3*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) - qp1*qpr3*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) - qp3*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qp3*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) - qp3*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) - qp1*qpr4*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qp1*qpr4*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) - qp1*qpr4*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) - qp4*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) - qp4*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) - qp4*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) - (qp1*qpr5*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/2 - (qp5*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/2 - (qp1*qpr6*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/2 - (qp6*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/2 - qp2*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp2*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp3*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp2*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp3*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp4*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp3*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp4*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - qp4*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*cos(q6) - (qp1*qpr5*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(q6))/2 - (qp5*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(q6))/2 + qp2*qpr2*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp2*qpr2*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp2*qpr2*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp2*qpr3*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp2*qpr3*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp2*qpr3*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp3*qpr2*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp3*qpr2*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp3*qpr2*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp2*qpr4*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp2*qpr4*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp2*qpr4*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp3*qpr3*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp3*qpr3*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp3*qpr3*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp4*qpr2*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp4*qpr2*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp4*qpr2*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp2*qpr5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp2*qpr5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp2*qpr5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qp3*qpr4*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp3*qpr4*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp3*qpr4*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp4*qpr3*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp4*qpr3*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp4*qpr3*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp5*qpr2*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp5*qpr2*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp5*qpr2*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + (qp2*qpr6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp2*qpr6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp2*qpr6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + qp3*qpr5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp3*qpr5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp3*qpr5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qp4*qpr4*cos(2*q5)*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qp4*qpr4*cos(2*q5)*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qp4*qpr4*cos(2*q5)*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qp5*qpr3*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp5*qpr3*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp5*qpr3*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + (qp6*qpr2*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr2*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr2*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp3*qpr6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp3*qpr6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp3*qpr6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + qp4*qpr5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp4*qpr5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp4*qpr5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qp5*qpr4*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp5*qpr4*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp5*qpr4*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + (qp6*qpr3*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr3*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr3*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp4*qpr6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp4*qpr6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp4*qpr6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr4*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr4*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr4*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2;



            Yr( 1,29) =(3*qppr1)/8 + (qppr1*cos(2*q5))/8 + (qppr1*cos(2*q6))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/8 - (qppr2*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (qppr2*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (qppr3*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (qppr3*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (qppr4*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (qppr4*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (qppr5*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (qppr5*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (qppr1*cos(2*q5 - 2*q6))/16 - (qppr1*cos(2*q5 + 2*q6))/16 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qppr2*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (qppr2*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (qppr2*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qppr2*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qppr2*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qppr2*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qppr3*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (qppr3*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (qppr3*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qppr3*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qppr3*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qppr3*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qppr4*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (qppr4*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (qppr4*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qppr4*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qppr4*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qppr4*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qppr5*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (qppr5*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qppr2*cos(q2 + q3 + q4 - 2*q5))/8 - (qppr2*cos(q2 + q3 + q4 + 2*q5))/8 + (qppr3*cos(q2 + q3 + q4 - 2*q5))/8 - (qppr3*cos(q2 + q3 + q4 + 2*q5))/8 + (qppr4*cos(q2 + q3 + q4 - 2*q5))/8 - (qppr4*cos(q2 + q3 + q4 + 2*q5))/8 - (qppr5*cos(q2 + q3 + q4 - 2*q6))/4 - (qppr5*cos(q2 + q3 + q4 + 2*q6))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 + (3*qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (qppr5*cos(q2 + q3 + q4))/2 - (qp2*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp2*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp2*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp2*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp3*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp3*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp2*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp2*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp3*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp3*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp4*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp4*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp3*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp3*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp4*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp4*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp2*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp2*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp4*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp4*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp6*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp6*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp2*qpr6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp2*qpr6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp2*qpr6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp2*qpr6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp3*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp3*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp6*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp6*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp6*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp6*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp6*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp6*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp3*qpr6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp3*qpr6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp3*qpr6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp3*qpr6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp4*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp4*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp5*qpr5*sin(q2 + q3 + q4 - q5 - 2*q6))/8 - (qp5*qpr5*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp6*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp6*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp6*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp6*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp6*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp6*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp4*qpr6*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp4*qpr6*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp4*qpr6*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp4*qpr6*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp5*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp5*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp6*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp6*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp6*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (qp6*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp6*qpr5*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp6*qpr5*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/8 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/8 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/8 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q6))/8 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q6))/8 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/8 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/8 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q6))/8 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q6))/8 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q6))/8 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q6))/8 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/8 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/8 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q6))/8 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q6))/8 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q6))/8 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q6))/8 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q6))/8 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q6))/8 - (qp5*qpr6*sin(q2 + q3 + q4 - 2*q6))/4 + (qp5*qpr6*sin(q2 + q3 + q4 + 2*q6))/4 - (qp6*qpr5*sin(q2 + q3 + q4 - 2*q6))/4 + (qp6*qpr5*sin(q2 + q3 + q4 + 2*q6))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 - (3*qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (3*qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (3*qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q6))/16 - (3*qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q6))/16 + (qp2*qpr5*sin(q2 + q3 + q4))/4 + (qp5*qpr2*sin(q2 + q3 + q4))/4 + (qp3*qpr5*sin(q2 + q3 + q4))/4 + (qp5*qpr3*sin(q2 + q3 + q4))/4 + (qp4*qpr5*sin(q2 + q3 + q4))/4 + (qp5*qpr4*sin(q2 + q3 + q4))/4 - (qp1*qpr5*sin(2*q5))/8 - (qp5*qpr1*sin(2*q5))/8 - (qp1*qpr6*sin(2*q6))/8 - (qp6*qpr1*sin(2*q6))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/8 + (qp2*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp2*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp2*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp2*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp3*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp3*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp2*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp2*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp3*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp3*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp4*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp4*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp3*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp3*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp4*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp4*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp2*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp2*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp4*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp4*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp6*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp6*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp3*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp3*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp6*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp6*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp4*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp4*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp5*qpr5*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp5*qpr5*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp6*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 - (qp6*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp5*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp5*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp6*qpr5*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp6*qpr5*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp1*qpr5*sin(2*q5 - 2*q6))/16 + (qp1*qpr5*sin(2*q5 + 2*q6))/16 + (qp5*qpr1*sin(2*q5 - 2*q6))/16 + (qp5*qpr1*sin(2*q5 + 2*q6))/16 - (qp1*qpr6*sin(2*q5 - 2*q6))/16 + (qp1*qpr6*sin(2*q5 + 2*q6))/16 - (qp6*qpr1*sin(2*q5 - 2*q6))/16 + (qp6*qpr1*sin(2*q5 + 2*q6))/16 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp1*qpr6*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/8 - (qp6*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/8;



            Yr( 1,30) =qppr6*cos(q2)*cos(q6)*sin(q3)*sin(q4) - (qp1*qpr5*cos(2*q5)*sin(q6))/2 - (qp5*qpr1*cos(2*q5)*sin(q6))/2 - (qp1*qpr6*sin(2*q5)*cos(q6))/4 - (qp6*qpr1*sin(2*q5)*cos(q6))/4 - qppr6*cos(q2)*cos(q3)*cos(q4)*cos(q6) - (qppr1*sin(2*q5)*sin(q6))/2 + qppr6*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qppr6*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qppr2*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qppr2*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qppr2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qppr3*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qppr3*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qppr3*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qppr4*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qppr4*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qppr4*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - qppr5*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) - qppr5*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - qppr5*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + qppr6*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qppr6*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qppr6*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + qppr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q6)*sin(q5) + qppr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q6)*sin(q5) + qppr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q6)*sin(q5) + qppr5*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - qppr6*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qppr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) + qppr2*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qppr2*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qppr2*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qppr3*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qppr3*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qppr3*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qppr4*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qppr4*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qppr4*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + (qppr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(q6))/2 - qppr2*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qppr3*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qppr4*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + (qp2*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp2*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp2*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr2*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr2*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr2*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp3*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp3*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp3*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr3*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr3*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr3*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp4*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp4*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp4*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr4*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr4*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr4*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + qp6*qpr6*cos(q2)*cos(q3)*cos(q4)*sin(q6) - (qppr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qppr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qppr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp2*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr2*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr3*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr4*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - qp6*qpr6*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp6*qpr6*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp6*qpr6*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qppr2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qppr3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - qppr4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - (qp1*qpr2*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qp1*qpr2*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp1*qpr2*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 - (qp2*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qp2*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp2*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 - (qp1*qpr3*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qp1*qpr3*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp1*qpr3*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 - (qp3*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qp3*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp3*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 - (qp1*qpr4*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qp1*qpr4*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp1*qpr4*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 - (qp4*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 - (qp4*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 - (qp4*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 - (qp1*qpr5*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(q6))/2 - (qp1*qpr5*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(q6))/2 - (qp1*qpr5*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(q6))/2 - (qp5*qpr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(q6))/2 - (qp5*qpr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(q6))/2 - (qp5*qpr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(q6))/2 - (qp1*qpr6*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/4 - (qp1*qpr6*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*cos(q6))/4 - (qp1*qpr6*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*cos(q6))/4 - (qp6*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/4 - (qp6*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*cos(q6))/4 - (qp6*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*cos(q6))/4 + qp2*qpr5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr2*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp3*qpr5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr3*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp4*qpr5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr4*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + (qp1*qpr2*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp2*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp1*qpr3*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp3*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp1*qpr4*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp4*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + qp2*qpr2*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp2*qpr2*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp2*qpr2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp2*qpr3*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp2*qpr3*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp2*qpr3*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp3*qpr2*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp3*qpr2*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp3*qpr2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp2*qpr4*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp2*qpr4*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp2*qpr4*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp3*qpr3*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp3*qpr3*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp3*qpr3*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp4*qpr2*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp4*qpr2*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp4*qpr2*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp3*qpr4*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp3*qpr4*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp3*qpr4*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp4*qpr3*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp4*qpr3*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp4*qpr3*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp2*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qp4*qpr4*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp4*qpr4*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp4*qpr4*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp6*qpr2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qp3*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qp6*qpr3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qp4*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - qp5*qpr5*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) - qp5*qpr5*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - qp5*qpr5*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp6*qpr4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qp6*qpr6*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) + qp6*qpr6*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + qp6*qpr6*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qp1*qpr2*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) + qp2*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) + qp1*qpr3*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) + qp3*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) + qp1*qpr4*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) + qp4*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) + (qp1*qpr5*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/2 + (qp1*qpr5*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*cos(q6))/2 + (qp1*qpr5*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*cos(q6))/2 + (qp5*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/2 + (qp5*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*cos(q6))/2 + (qp5*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*cos(q6))/2 - qp2*qpr2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp2*qpr3*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp3*qpr2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp2*qpr4*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp3*qpr3*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp4*qpr2*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp3*qpr4*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp4*qpr3*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp2*qpr6*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr6*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - qp2*qpr6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qp4*qpr4*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp6*qpr2*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - qp6*qpr2*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - qp6*qpr2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qp3*qpr6*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr6*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - qp3*qpr6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qp6*qpr3*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - qp6*qpr3*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - qp6*qpr3*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qp4*qpr6*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr6*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - qp4*qpr6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + qp5*qpr5*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp6*qpr4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - qp6*qpr4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - qp6*qpr4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qp6*qpr6*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp1*qpr2*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) - qp1*qpr2*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) - qp1*qpr2*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) - qp2*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) - qp2*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) - qp2*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) - qp1*qpr3*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) - qp1*qpr3*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) - qp1*qpr3*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) - qp3*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) - qp3*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) - qp3*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) - qp1*qpr4*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) - qp1*qpr4*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) - qp1*qpr4*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) - qp4*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) - qp4*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) - qp4*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) - (qp1*qpr5*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/2 - (qp5*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/2 - (qp1*qpr6*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/2 - (qp1*qpr6*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5)*sin(q6))/2 - (qp1*qpr6*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5)*sin(q6))/2 - (qp6*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/2 - (qp6*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5)*sin(q6))/2 - (qp6*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5)*sin(q6))/2 + (qp1*qpr6*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/2 + (qp6*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/2 + qp2*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp2*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp3*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp2*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp3*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp4*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp3*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qp4*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + (qp2*qpr6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp2*qpr6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp2*qpr6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + qp4*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q4)*sin(q6) + (qp6*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr2*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr2*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp3*qpr6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp3*qpr6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp3*qpr6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr3*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr3*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp4*qpr6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp4*qpr6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp4*qpr6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr4*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr4*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp1*qpr5*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(q6))/2 + (qp5*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(q6))/2 + (qp1*qpr6*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*cos(q6))/4 + (qp6*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*cos(q6))/4 - qp2*qpr2*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr2*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp2*qpr2*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp2*qpr3*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr3*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp2*qpr3*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp3*qpr2*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr2*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp3*qpr2*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp2*qpr4*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr4*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp2*qpr4*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp3*qpr3*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr3*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp3*qpr3*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp4*qpr2*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr2*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp4*qpr2*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp2*qpr5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp2*qpr5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp2*qpr5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qp3*qpr4*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr4*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp3*qpr4*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp4*qpr3*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr3*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp4*qpr3*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp5*qpr2*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp5*qpr2*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp5*qpr2*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - (qp2*qpr6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - qp3*qpr5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp3*qpr5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp3*qpr5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qp4*qpr4*cos(2*q5)*cos(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr4*cos(2*q5)*cos(q3)*sin(q2)*sin(q4)*sin(q6) - qp4*qpr4*cos(2*q5)*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qp5*qpr3*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp5*qpr3*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp5*qpr3*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - (qp6*qpr2*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - qp4*qpr5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp4*qpr5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp4*qpr5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qp5*qpr4*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp5*qpr4*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp5*qpr4*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - (qp6*qpr3*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr4*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 1,31) =qppr1/4 - (qppr1*cos(2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4))/4 + (qppr6*cos(q2 + q3 + q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr2*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr3*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr3*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr4*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr4*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr6*cos(q2 + q3 + q4 - q5))/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr6*sin(q2 + q3 + q4 - q5))/4 + (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp6*qpr2*sin(q2 + q3 + q4 - q5))/4 - (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr6*sin(q2 + q3 + q4 - q5))/4 + (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp6*qpr3*sin(q2 + q3 + q4 - q5))/4 - (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr6*sin(q2 + q3 + q4 - q5))/4 - (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp6*qpr4*sin(q2 + q3 + q4 - q5))/4 - (qp5*qpr6*sin(q2 + q3 + q4 - q5))/4 - (qp6*qpr5*sin(q2 + q3 + q4 - q5))/4 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr5*sin(2*q5))/4 + (qp5*qpr1*sin(2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp2*qpr6*sin(q2 + q3 + q4 + q5))/4 - (qp6*qpr2*sin(q2 + q3 + q4 + q5))/4 - (qp3*qpr6*sin(q2 + q3 + q4 + q5))/4 - (qp6*qpr3*sin(q2 + q3 + q4 + q5))/4 - (qp4*qpr6*sin(q2 + q3 + q4 + q5))/4 - (qp6*qpr4*sin(q2 + q3 + q4 + q5))/4 - (qp5*qpr6*sin(q2 + q3 + q4 + q5))/4 - (qp6*qpr5*sin(q2 + q3 + q4 + q5))/4;



            Yr( 1,32) =(3*qppr1)/4 + (qppr1*cos(2*q5))/4 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/4 + (qppr2*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr2*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr3*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr3*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr4*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr4*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - qppr5*cos(q2 + q3 + q4) - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr2*sin(q2 + q3 + q4))/2 + (qp3*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr3*sin(q2 + q3 + q4))/2 + (qp4*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr4*sin(q2 + q3 + q4))/2 - (qp1*qpr5*sin(2*q5))/4 - (qp5*qpr1*sin(2*q5))/4 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 1,33) =qppr1;



            Yr( 1,34) =qppr1;



            Yr( 1,35) =qppr1/2 + (qppr1*cos(2*q2))/2 - (qp1*qpr2*sin(2*q2))/2 - (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,36) =qppr1;



            Yr( 1,37) =qppr1/2 + (qppr1*cos(2*q2))/2 - (qp1*qpr2*sin(2*q2))/2 - (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,38) =qppr1;



            Yr( 1,39) =qppr1;



            Yr( 1,40) =qppr1/2 + (qppr1*cos(2*q2))/2 - (qp1*qpr2*sin(2*q2))/2 - (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,41) =qppr1/2 + (qppr1*cos(2*q2))/2 - (qp1*qpr2*sin(2*q2))/2 - (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,42) =qppr1/2 + (qppr1*cos(2*q2 + 2*q3))/2 - (qp1*qpr2*sin(2*q2 + 2*q3))/2 - (qp2*qpr1*sin(2*q2 + 2*q3))/2 - (qp1*qpr3*sin(2*q2 + 2*q3))/2 - (qp3*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 1,43) =qppr1;



            Yr( 1,44) =(3*qppr1)/4 + (qppr1*cos(2*q5))/4 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/4 + (qppr2*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr2*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr3*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr3*cos(q2 + q3 + q4 + 2*q5))/4 + (qppr4*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr4*cos(q2 + q3 + q4 + 2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - qppr5*cos(q2 + q3 + q4) - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr2*sin(q2 + q3 + q4))/2 + (qp3*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr3*sin(q2 + q3 + q4))/2 + (qp4*qpr5*sin(q2 + q3 + q4))/2 + (qp5*qpr4*sin(q2 + q3 + q4))/2 - (qp1*qpr5*sin(2*q5))/4 - (qp5*qpr1*sin(2*q5))/4 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/4 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 1,45) =qppr1/2 + (qppr1*cos(2*q2 + 2*q3))/2 - (qp1*qpr2*sin(2*q2 + 2*q3))/2 - (qp2*qpr1*sin(2*q2 + 2*q3))/2 - (qp1*qpr3*sin(2*q2 + 2*q3))/2 - (qp3*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 1,46) =qppr1/2 + (qppr1*cos(2*q2))/2 - (qp1*qpr2*sin(2*q2))/2 - (qp2*qpr1*sin(2*q2))/2;



            Yr( 1,47) =qppr1/2 + (qppr1*cos(2*q2 + 2*q3))/2 - (qp1*qpr2*sin(2*q2 + 2*q3))/2 - (qp2*qpr1*sin(2*q2 + 2*q3))/2 - (qp1*qpr3*sin(2*q2 + 2*q3))/2 - (qp3*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 1,48) =qppr1/2 + (qppr1*cos(2*q2 + 2*q3))/2 - (qp1*qpr2*sin(2*q2 + 2*q3))/2 - (qp2*qpr1*sin(2*q2 + 2*q3))/2 - (qp1*qpr3*sin(2*q2 + 2*q3))/2 - (qp3*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 1,49) =qppr1/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 1,50) =qppr1;



            Yr( 1,51) =qppr1;



            Yr( 1,52) =qppr1;



            Yr( 1,53) =qppr1;



            Yr( 1,54) =qppr1;



            Yr( 1,55) =cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5);



            Yr( 1,56) =cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q5)*sin(q1) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5);



            Yr( 1,57) =0;



            Yr( 1,58) =2*qppr1*cos(q5) - (qppr3*cos(q2 + q3 + q4 + q5))/2 - (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr5*cos(q2 + q3 + q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr2*cos(q2 + q3 + q4 - q5))/2 + (qppr3*cos(q2 + q3 + q4 - q5))/2 + (qppr4*cos(q2 + q3 + q4 - q5))/2 - (qppr5*cos(q2 + q3 + q4 - q5))/2 - qp1*qpr5*sin(q5) - qp5*qpr1*sin(q5) - (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2;



            Yr( 1,59) =(qppr2*sin(q2 + q5))/2 - (qppr1*sin(q3 + q4 - q5))/2 - (qppr1*sin(2*q2 + q3 + q4 - q5))/2 - (qppr5*sin(q2 + q5))/2 + (qppr1*sin(2*q2 + q3 + q4 + q5))/2 + (qppr2*sin(q2 - q5))/2 + (qppr5*sin(q2 - q5))/2 + (qppr1*sin(q3 + q4 + q5))/2 + (qp1*qpr2*cos(2*q2 + q3 + q4 + q5))/2 + (qp2*qpr1*cos(2*q2 + q3 + q4 + q5))/2 + (qp1*qpr3*cos(2*q2 + q3 + q4 + q5))/4 + (qp3*qpr1*cos(2*q2 + q3 + q4 + q5))/4 + (qp1*qpr4*cos(2*q2 + q3 + q4 + q5))/4 + (qp4*qpr1*cos(2*q2 + q3 + q4 + q5))/4 + (qp1*qpr5*cos(2*q2 + q3 + q4 + q5))/4 + (qp5*qpr1*cos(2*q2 + q3 + q4 + q5))/4 + (qp2*qpr2*cos(q2 - q5))/2 - (qp5*qpr5*cos(q2 - q5))/2 + (qp1*qpr3*cos(q3 + q4 + q5))/4 + (qp3*qpr1*cos(q3 + q4 + q5))/4 + (qp1*qpr4*cos(q3 + q4 + q5))/4 + (qp4*qpr1*cos(q3 + q4 + q5))/4 + (qp1*qpr5*cos(q3 + q4 + q5))/4 + (qp5*qpr1*cos(q3 + q4 + q5))/4 - (qp1*qpr2*cos(2*q2 + q3 + q4 - q5))/2 - (qp2*qpr1*cos(2*q2 + q3 + q4 - q5))/2 - (qp1*qpr3*cos(2*q2 + q3 + q4 - q5))/4 - (qp3*qpr1*cos(2*q2 + q3 + q4 - q5))/4 - (qp1*qpr4*cos(2*q2 + q3 + q4 - q5))/4 - (qp4*qpr1*cos(2*q2 + q3 + q4 - q5))/4 + (qp1*qpr5*cos(2*q2 + q3 + q4 - q5))/4 + (qp5*qpr1*cos(2*q2 + q3 + q4 - q5))/4 - (qp1*qpr3*cos(q3 + q4 - q5))/4 - (qp3*qpr1*cos(q3 + q4 - q5))/4 - (qp1*qpr4*cos(q3 + q4 - q5))/4 - (qp4*qpr1*cos(q3 + q4 - q5))/4 + (qp1*qpr5*cos(q3 + q4 - q5))/4 + (qp5*qpr1*cos(q3 + q4 - q5))/4 + (qp2*qpr2*cos(q2 + q5))/2 - (qp5*qpr5*cos(q2 + q5))/2;



            Yr( 1,60) =(3*qppr1)/2 + (qppr1*cos(2*q5))/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 + (qppr2*cos(q2 + q3 + q4 - 2*q5))/2 - (qppr2*cos(q2 + q3 + q4 + 2*q5))/2 + (qppr3*cos(q2 + q3 + q4 - 2*q5))/2 - (qppr3*cos(q2 + q3 + q4 + 2*q5))/2 + (qppr4*cos(q2 + q3 + q4 - 2*q5))/2 - (qppr4*cos(q2 + q3 + q4 + 2*q5))/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - 2*qppr5*cos(q2 + q3 + q4) - (qp2*qpr2*sin(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr2*sin(q2 + q3 + q4 + 2*q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 + 2*q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 + 2*q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 + 2*q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 + 2*q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 + 2*q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 - 2*q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 + 2*q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 + 2*q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 + 2*q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 + 2*q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 - 2*q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 + 2*q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 + 2*q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 + 2*q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 - 2*q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 + 2*q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 + (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + qp2*qpr5*sin(q2 + q3 + q4) + qp5*qpr2*sin(q2 + q3 + q4) + qp3*qpr5*sin(q2 + q3 + q4) + qp5*qpr3*sin(q2 + q3 + q4) + qp4*qpr5*sin(q2 + q3 + q4) + qp5*qpr4*sin(q2 + q3 + q4) - (qp1*qpr5*sin(2*q5))/2 - (qp5*qpr1*sin(2*q5))/2 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 1,61) =(qppr1*sin(2*q2 + 2*q3 + q4 + q5))/2 + (qppr2*sin(q2 + q3 - q5))/2 + (qppr3*sin(q2 + q3 - q5))/2 + (qppr5*sin(q2 + q3 - q5))/2 + (qppr1*sin(q4 + q5))/2 - (qppr1*sin(2*q2 + 2*q3 + q4 - q5))/2 - (qppr1*sin(q4 - q5))/2 + (qppr2*sin(q2 + q3 + q5))/2 + (qppr3*sin(q2 + q3 + q5))/2 - (qppr5*sin(q2 + q3 + q5))/2 - (qp1*qpr4*cos(q4 - q5))/4 - (qp4*qpr1*cos(q4 - q5))/4 + (qp1*qpr5*cos(q4 - q5))/4 + (qp5*qpr1*cos(q4 - q5))/4 + (qp2*qpr2*cos(q2 + q3 + q5))/2 + (qp2*qpr3*cos(q2 + q3 + q5))/2 + (qp3*qpr2*cos(q2 + q3 + q5))/2 + (qp3*qpr3*cos(q2 + q3 + q5))/2 - (qp5*qpr5*cos(q2 + q3 + q5))/2 + (qp1*qpr2*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp2*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp1*qpr3*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp3*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp1*qpr4*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp4*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp1*qpr5*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp2*qpr2*cos(q2 + q3 - q5))/2 + (qp2*qpr3*cos(q2 + q3 - q5))/2 + (qp3*qpr2*cos(q2 + q3 - q5))/2 + (qp3*qpr3*cos(q2 + q3 - q5))/2 - (qp5*qpr5*cos(q2 + q3 - q5))/2 + (qp1*qpr4*cos(q4 + q5))/4 + (qp4*qpr1*cos(q4 + q5))/4 + (qp1*qpr5*cos(q4 + q5))/4 + (qp5*qpr1*cos(q4 + q5))/4 - (qp1*qpr2*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp2*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp1*qpr3*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp3*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp1*qpr4*cos(2*q2 + 2*q3 + q4 - q5))/4 - (qp4*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/4 + (qp1*qpr5*cos(2*q2 + 2*q3 + q4 - q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/4;



            Yr( 1,62) =(qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr3*cos(q2 + q3 + q4 + q5))/2 + (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr5*cos(q2 + q3 + q4 + q5))/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qppr2*cos(q2 + q3 + q4 - q5))/2 + (qppr3*cos(q2 + q3 + q4 - q5))/2 + (qppr4*cos(q2 + q3 + q4 - q5))/2 + (qppr5*cos(q2 + q3 + q4 - q5))/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 - (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5))/4 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 1,63) =2*qppr1*cos(q5) - (qppr3*cos(q2 + q3 + q4 + q5))/2 - (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr5*cos(q2 + q3 + q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr2*cos(q2 + q3 + q4 - q5))/2 + (qppr3*cos(q2 + q3 + q4 - q5))/2 + (qppr4*cos(q2 + q3 + q4 - q5))/2 - (qppr5*cos(q2 + q3 + q4 - q5))/2 - qp1*qpr5*sin(q5) - qp5*qpr1*sin(q5) - (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2;



            Yr( 1,64) =(qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr3*cos(q2 + q3 + q4 + q5))/2 + (qppr4*cos(q2 + q3 + q4 + q5))/2 + (qppr5*cos(q2 + q3 + q4 + q5))/2 - 2*qppr1*cos(q5) - (qppr2*cos(q2 + q3 + q4 - q5))/2 - (qppr3*cos(q2 + q3 + q4 - q5))/2 - (qppr4*cos(q2 + q3 + q4 - q5))/2 + (qppr5*cos(q2 + q3 + q4 - q5))/2 + qp1*qpr5*sin(q5) + qp5*qpr1*sin(q5) + (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2;



            Yr( 1,65) =-cos(q1);



            Yr( 1,66) =-cos(q1);



            Yr( 1,67) =-cos(q2)*sin(q1);



            Yr( 1,68) =-cos(q1);



            Yr( 1,69) =-cos(q2)*sin(q1);



            Yr( 1,70) =cos(q1);



            Yr( 1,71) =-cos(q1);



            Yr( 1,72) =-cos(q2)*sin(q1);



            Yr( 1,73) =-cos(q2)*sin(q1);



            Yr( 1,74) =-cos(q2 + q3)*sin(q1);



            Yr( 1,75) =-cos(q1);



            Yr( 1,76) =cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5);



            Yr( 1,77) =-cos(q2 + q3)*sin(q1);



            Yr( 1,78) =-cos(q2)*sin(q1);



            Yr( 1,79) =-cos(q2 + q3)*sin(q1);



            Yr( 1,80) =-cos(q2 + q3)*sin(q1);



            Yr( 1,81) =cos(q2 - q1 + q3 + q4)/2 - cos(q1 + q2 + q3 + q4)/2;



            Yr( 1,82) =-cos(q1);



            Yr( 1,83) =cos(q1);



            Yr( 1,84) =-cos(q1);



            Yr( 1,85) =cos(q1);



            Yr( 1,86) =cos(q1);



            Yr( 1,87) =-sin(q1);



            Yr( 1,88) =-sin(q1);



            Yr( 1,89) =cos(q1)*cos(q2);



            Yr( 1,90) =-sin(q1);



            Yr( 1,91) =cos(q1)*cos(q2);



            Yr( 1,92) =sin(q1);



            Yr( 1,93) =-sin(q1);



            Yr( 1,94) =cos(q1)*cos(q2);



            Yr( 1,95) =cos(q1)*cos(q2);



            Yr( 1,96) =cos(q2 + q3)*cos(q1);



            Yr( 1,97) =-sin(q1);



            Yr( 1,98) =cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q5)*sin(q1) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5);



            Yr( 1,99) =cos(q2 + q3)*cos(q1);



            Yr( 1,100) =cos(q1)*cos(q2);



            Yr( 1,101) =cos(q2 + q3)*cos(q1);



            Yr( 1,102) =cos(q2 + q3)*cos(q1);



            Yr( 1,103) =- sin(q1 + q2 + q3 + q4)/2 - sin(q2 - q1 + q3 + q4)/2;



            Yr( 1,104) =-sin(q1);



            Yr( 1,105) =sin(q1);



            Yr( 1,106) =-sin(q1);



            Yr( 1,107) =sin(q1);



            Yr( 1,108) =sin(q1);



            Yr( 1,109) =0;



            Yr( 1,110) =0;



            Yr( 1,111) =0;



            Yr( 1,112) =0;



            Yr( 1,113) =0;



            Yr( 1,114) =0;



            Yr( 1,115) =0;



            Yr( 1,116) =0;



            Yr( 1,117) =0;



            Yr( 1,118) =0;



            Yr( 1,119) =0;



            Yr( 1,120) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,121) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,122) =-2*qppr1;



            Yr( 1,123) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,124) =- qppr2*sin(q2) - qp2*qpr2*cos(q2);



            Yr( 1,125) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,126) =qppr2*sin(q2 + q3) + qppr3*sin(q2 + q3) + qp2*qpr2*cos(q2 + q3) + qp2*qpr3*cos(q2 + q3) + qp3*qpr2*cos(q2 + q3) + qp3*qpr3*cos(q2 + q3);



            Yr( 1,127) =2*qppr1*cos(q5) - (qppr3*cos(q2 + q3 + q4 + q5))/2 - (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr5*cos(q2 + q3 + q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr2*cos(q2 + q3 + q4 - q5))/2 + (qppr3*cos(q2 + q3 + q4 - q5))/2 + (qppr4*cos(q2 + q3 + q4 - q5))/2 - (qppr5*cos(q2 + q3 + q4 - q5))/2 - qp1*qpr5*sin(q5) - qp5*qpr1*sin(q5) - (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2;



            Yr( 1,128) =qppr2*sin(q2 + q3) + qppr3*sin(q2 + q3) + qp2*qpr2*cos(q2 + q3) + qp2*qpr3*cos(q2 + q3) + qp3*qpr2*cos(q2 + q3) + qp3*qpr3*cos(q2 + q3);



            Yr( 1,129) =qppr1*cos(q3) + qppr1*cos(2*q2 + q3) - (qp1*qpr3*sin(q3))/2 - (qp3*qpr1*sin(q3))/2 - qp1*qpr2*sin(2*q2 + q3) - qp2*qpr1*sin(2*q2 + q3) - (qp1*qpr3*sin(2*q2 + q3))/2 - (qp3*qpr1*sin(2*q2 + q3))/2;



            Yr( 1,130) =qppr2*sin(q2 + q3) + qppr3*sin(q2 + q3) + qp2*qpr2*cos(q2 + q3) + qp2*qpr3*cos(q2 + q3) + qp3*qpr2*cos(q2 + q3) + qp3*qpr3*cos(q2 + q3);



            Yr( 1,131) =(qppr2*sin(q2 + q5))/2 - (qppr1*sin(q3 + q4 - q5))/2 - (qppr1*sin(2*q2 + q3 + q4 - q5))/2 - (qppr5*sin(q2 + q5))/2 + (qppr1*sin(2*q2 + q3 + q4 + q5))/2 + (qppr2*sin(q2 - q5))/2 + (qppr5*sin(q2 - q5))/2 + (qppr1*sin(q3 + q4 + q5))/2 + (qp1*qpr2*cos(2*q2 + q3 + q4 + q5))/2 + (qp2*qpr1*cos(2*q2 + q3 + q4 + q5))/2 + (qp1*qpr3*cos(2*q2 + q3 + q4 + q5))/4 + (qp3*qpr1*cos(2*q2 + q3 + q4 + q5))/4 + (qp1*qpr4*cos(2*q2 + q3 + q4 + q5))/4 + (qp4*qpr1*cos(2*q2 + q3 + q4 + q5))/4 + (qp1*qpr5*cos(2*q2 + q3 + q4 + q5))/4 + (qp5*qpr1*cos(2*q2 + q3 + q4 + q5))/4 + (qp2*qpr2*cos(q2 - q5))/2 - (qp5*qpr5*cos(q2 - q5))/2 + (qp1*qpr3*cos(q3 + q4 + q5))/4 + (qp3*qpr1*cos(q3 + q4 + q5))/4 + (qp1*qpr4*cos(q3 + q4 + q5))/4 + (qp4*qpr1*cos(q3 + q4 + q5))/4 + (qp1*qpr5*cos(q3 + q4 + q5))/4 + (qp5*qpr1*cos(q3 + q4 + q5))/4 - (qp1*qpr2*cos(2*q2 + q3 + q4 - q5))/2 - (qp2*qpr1*cos(2*q2 + q3 + q4 - q5))/2 - (qp1*qpr3*cos(2*q2 + q3 + q4 - q5))/4 - (qp3*qpr1*cos(2*q2 + q3 + q4 - q5))/4 - (qp1*qpr4*cos(2*q2 + q3 + q4 - q5))/4 - (qp4*qpr1*cos(2*q2 + q3 + q4 - q5))/4 + (qp1*qpr5*cos(2*q2 + q3 + q4 - q5))/4 + (qp5*qpr1*cos(2*q2 + q3 + q4 - q5))/4 - (qp1*qpr3*cos(q3 + q4 - q5))/4 - (qp3*qpr1*cos(q3 + q4 - q5))/4 - (qp1*qpr4*cos(q3 + q4 - q5))/4 - (qp4*qpr1*cos(q3 + q4 - q5))/4 + (qp1*qpr5*cos(q3 + q4 - q5))/4 + (qp5*qpr1*cos(q3 + q4 - q5))/4 + (qp2*qpr2*cos(q2 + q5))/2 - (qp5*qpr5*cos(q2 + q5))/2;



            Yr( 1,132) =qppr1*cos(q3) + qppr1*cos(2*q2 + q3) - (qp1*qpr3*sin(q3))/2 - (qp3*qpr1*sin(q3))/2 - qp1*qpr2*sin(2*q2 + q3) - qp2*qpr1*sin(2*q2 + q3) - (qp1*qpr3*sin(2*q2 + q3))/2 - (qp3*qpr1*sin(2*q2 + q3))/2;



            Yr( 1,133) =qppr1*cos(q3) + qppr1*cos(2*q2 + q3) - (qp1*qpr3*sin(q3))/2 - (qp3*qpr1*sin(q3))/2 - qp1*qpr2*sin(2*q2 + q3) - qp2*qpr1*sin(2*q2 + q3) - (qp1*qpr3*sin(2*q2 + q3))/2 - (qp3*qpr1*sin(2*q2 + q3))/2;



            Yr( 1,134) =qppr2*sin(q2 + q3) + qppr3*sin(q2 + q3) + qp2*qpr2*cos(q2 + q3) + qp2*qpr3*cos(q2 + q3) + qp3*qpr2*cos(q2 + q3) + qp3*qpr3*cos(q2 + q3);



            Yr( 1,135) =(qppr1*sin(2*q2 + 2*q3 + q4 + q5))/2 + (qppr2*sin(q2 + q3 - q5))/2 + (qppr3*sin(q2 + q3 - q5))/2 + (qppr5*sin(q2 + q3 - q5))/2 + (qppr1*sin(q4 + q5))/2 - (qppr1*sin(2*q2 + 2*q3 + q4 - q5))/2 - (qppr1*sin(q4 - q5))/2 + (qppr2*sin(q2 + q3 + q5))/2 + (qppr3*sin(q2 + q3 + q5))/2 - (qppr5*sin(q2 + q3 + q5))/2 - (qp1*qpr4*cos(q4 - q5))/4 - (qp4*qpr1*cos(q4 - q5))/4 + (qp1*qpr5*cos(q4 - q5))/4 + (qp5*qpr1*cos(q4 - q5))/4 + (qp2*qpr2*cos(q2 + q3 + q5))/2 + (qp2*qpr3*cos(q2 + q3 + q5))/2 + (qp3*qpr2*cos(q2 + q3 + q5))/2 + (qp3*qpr3*cos(q2 + q3 + q5))/2 - (qp5*qpr5*cos(q2 + q3 + q5))/2 + (qp1*qpr2*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp2*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp1*qpr3*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp3*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/2 + (qp1*qpr4*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp4*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp1*qpr5*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + q4 + q5))/4 + (qp2*qpr2*cos(q2 + q3 - q5))/2 + (qp2*qpr3*cos(q2 + q3 - q5))/2 + (qp3*qpr2*cos(q2 + q3 - q5))/2 + (qp3*qpr3*cos(q2 + q3 - q5))/2 - (qp5*qpr5*cos(q2 + q3 - q5))/2 + (qp1*qpr4*cos(q4 + q5))/4 + (qp4*qpr1*cos(q4 + q5))/4 + (qp1*qpr5*cos(q4 + q5))/4 + (qp5*qpr1*cos(q4 + q5))/4 - (qp1*qpr2*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp2*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp1*qpr3*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp3*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/2 - (qp1*qpr4*cos(2*q2 + 2*q3 + q4 - q5))/4 - (qp4*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/4 + (qp1*qpr5*cos(2*q2 + 2*q3 + q4 - q5))/4 + (qp5*qpr1*cos(2*q2 + 2*q3 + q4 - q5))/4;



            Yr( 1,136) =qppr1*cos(q3) + qppr1*cos(2*q2 + q3) - (qp1*qpr3*sin(q3))/2 - (qp3*qpr1*sin(q3))/2 - qp1*qpr2*sin(2*q2 + q3) - qp2*qpr1*sin(2*q2 + q3) - (qp1*qpr3*sin(2*q2 + q3))/2 - (qp3*qpr1*sin(2*q2 + q3))/2;



            Yr( 1,137) =qppr2*cos(q2 + q3 + q4) + qppr3*cos(q2 + q3 + q4) + qppr4*cos(q2 + q3 + q4) - qp2*qpr2*sin(q2 + q3 + q4) - qp2*qpr3*sin(q2 + q3 + q4) - qp3*qpr2*sin(q2 + q3 + q4) - qp2*qpr4*sin(q2 + q3 + q4) - qp3*qpr3*sin(q2 + q3 + q4) - qp4*qpr2*sin(q2 + q3 + q4) - qp3*qpr4*sin(q2 + q3 + q4) - qp4*qpr3*sin(q2 + q3 + q4) - qp4*qpr4*sin(q2 + q3 + q4);



            Yr( 1,138) =- qppr2*sin(q2 + q3) - qppr3*sin(q2 + q3) - qp2*qpr2*cos(q2 + q3) - qp2*qpr3*cos(q2 + q3) - qp3*qpr2*cos(q2 + q3) - qp3*qpr3*cos(q2 + q3);



            Yr( 1,139) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,140) =2*qppr1;



            Yr( 1,141) =-2*qppr1;



            Yr( 1,142) =- qppr1*sin(2*q2 + q3 + q4) - qppr1*sin(q3 + q4) - qp1*qpr2*cos(2*q2 + q3 + q4) - qp2*qpr1*cos(2*q2 + q3 + q4) - (qp1*qpr3*cos(2*q2 + q3 + q4))/2 - (qp3*qpr1*cos(2*q2 + q3 + q4))/2 - (qp1*qpr4*cos(2*q2 + q3 + q4))/2 - (qp4*qpr1*cos(2*q2 + q3 + q4))/2 - (qp1*qpr3*cos(q3 + q4))/2 - (qp3*qpr1*cos(q3 + q4))/2 - (qp1*qpr4*cos(q3 + q4))/2 - (qp4*qpr1*cos(q3 + q4))/2;



            Yr( 1,143) =2*qppr1;



            Yr( 1,144) =-2*qppr1;



            Yr( 1,145) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,146) =- qppr2*sin(q2) - qp2*qpr2*cos(q2);



            Yr( 1,147) =(qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr3*cos(q2 + q3 + q4 + q5))/2 + (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr5*cos(q2 + q3 + q4 + q5))/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2 + (qppr2*cos(q2 + q3 + q4 - q5))/2 + (qppr3*cos(q2 + q3 + q4 - q5))/2 + (qppr4*cos(q2 + q3 + q4 - q5))/2 + (qppr5*cos(q2 + q3 + q4 - q5))/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 - (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr2*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp2*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2 - (qp1*qpr5*sin(2*q2 + 2*q3 + 2*q4 + q5))/4 - (qp5*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 1,148) =-2*qppr1;



            Yr( 1,149) =qppr2*sin(q2) + qp2*qpr2*cos(q2);



            Yr( 1,150) =- qppr2*sin(q2) - qp2*qpr2*cos(q2);



            Yr( 1,151) =- qppr1*sin(q4) - qppr1*sin(2*q2 + 2*q3 + q4) - (qp1*qpr4*cos(q4))/2 - (qp4*qpr1*cos(q4))/2 - qp1*qpr2*cos(2*q2 + 2*q3 + q4) - qp2*qpr1*cos(2*q2 + 2*q3 + q4) - qp1*qpr3*cos(2*q2 + 2*q3 + q4) - qp3*qpr1*cos(2*q2 + 2*q3 + q4) - (qp1*qpr4*cos(2*q2 + 2*q3 + q4))/2 - (qp4*qpr1*cos(2*q2 + 2*q3 + q4))/2;



            Yr( 1,152) =- qppr2*sin(q2) - qp2*qpr2*cos(q2);



            Yr( 1,153) =2*qppr1*cos(q5) - (qppr3*cos(q2 + q3 + q4 + q5))/2 - (qppr4*cos(q2 + q3 + q4 + q5))/2 - (qppr5*cos(q2 + q3 + q4 + q5))/2 - (qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr2*cos(q2 + q3 + q4 - q5))/2 + (qppr3*cos(q2 + q3 + q4 - q5))/2 + (qppr4*cos(q2 + q3 + q4 - q5))/2 - (qppr5*cos(q2 + q3 + q4 - q5))/2 - qp1*qpr5*sin(q5) - qp5*qpr1*sin(q5) - (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2;



            Yr( 1,154) =qppr2*sin(q2 + q3) + qppr3*sin(q2 + q3) + qp2*qpr2*cos(q2 + q3) + qp2*qpr3*cos(q2 + q3) + qp3*qpr2*cos(q2 + q3) + qp3*qpr3*cos(q2 + q3);



            Yr( 1,155) =- qppr2*sin(q2 + q3) - qppr3*sin(q2 + q3) - qp2*qpr2*cos(q2 + q3) - qp2*qpr3*cos(q2 + q3) - qp3*qpr2*cos(q2 + q3) - qp3*qpr3*cos(q2 + q3);



            Yr( 1,156) =(qppr2*cos(q2 + q3 + q4 + q5))/2 + (qppr3*cos(q2 + q3 + q4 + q5))/2 + (qppr4*cos(q2 + q3 + q4 + q5))/2 + (qppr5*cos(q2 + q3 + q4 + q5))/2 - 2*qppr1*cos(q5) - (qppr2*cos(q2 + q3 + q4 - q5))/2 - (qppr3*cos(q2 + q3 + q4 - q5))/2 - (qppr4*cos(q2 + q3 + q4 - q5))/2 + (qppr5*cos(q2 + q3 + q4 - q5))/2 + qp1*qpr5*sin(q5) + qp5*qpr1*sin(q5) + (qp2*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr5*sin(q2 + q3 + q4 + q5))/2;



            Yr( 1,157) =qppr2*sin(q2 + q3) + qppr3*sin(q2 + q3) + qp2*qpr2*cos(q2 + q3) + qp2*qpr3*cos(q2 + q3) + qp3*qpr2*cos(q2 + q3) + qp3*qpr3*cos(q2 + q3);



            Yr( 1,158) =- qppr2*sin(q2 + q3) - qppr3*sin(q2 + q3) - qp2*qpr2*cos(q2 + q3) - qp2*qpr3*cos(q2 + q3) - qp3*qpr2*cos(q2 + q3) - qp3*qpr3*cos(q2 + q3);



            Yr( 1,159) =- qppr2*sin(q2 + q3) - qppr3*sin(q2 + q3) - qp2*qpr2*cos(q2 + q3) - qp2*qpr3*cos(q2 + q3) - qp3*qpr2*cos(q2 + q3) - qp3*qpr3*cos(q2 + q3);



            Yr( 1,160) =qppr2*cos(q2 + q3 + q4) + qppr3*cos(q2 + q3 + q4) + qppr4*cos(q2 + q3 + q4) - qp2*qpr2*sin(q2 + q3 + q4) - qp2*qpr3*sin(q2 + q3 + q4) - qp3*qpr2*sin(q2 + q3 + q4) - qp2*qpr4*sin(q2 + q3 + q4) - qp3*qpr3*sin(q2 + q3 + q4) - qp4*qpr2*sin(q2 + q3 + q4) - qp3*qpr4*sin(q2 + q3 + q4) - qp4*qpr3*sin(q2 + q3 + q4) - qp4*qpr4*sin(q2 + q3 + q4);



            Yr( 1,161) =qp2*qpr2*sin(q2 + q3 + q4) - qppr3*cos(q2 + q3 + q4) - qppr4*cos(q2 + q3 + q4) - qppr2*cos(q2 + q3 + q4) + qp2*qpr3*sin(q2 + q3 + q4) + qp3*qpr2*sin(q2 + q3 + q4) + qp2*qpr4*sin(q2 + q3 + q4) + qp3*qpr3*sin(q2 + q3 + q4) + qp4*qpr2*sin(q2 + q3 + q4) + qp3*qpr4*sin(q2 + q3 + q4) + qp4*qpr3*sin(q2 + q3 + q4) + qp4*qpr4*sin(q2 + q3 + q4);



            Yr( 1,162) =-2*qppr1;



            Yr( 1,163) =-2*qppr1;



            Yr( 2,1) =0;



            Yr( 2,2) =-(qp1*qpr1*sin(2*q2))/2;



            Yr( 2,3) =-qp1*qpr1*cos(2*q2);



            Yr( 2,4) =-qppr1*sin(q2);



            Yr( 2,5) =(qp1*qpr1*sin(2*q2))/2;



            Yr( 2,6) =-qppr1*cos(q2);



            Yr( 2,7) =qppr2;



            Yr( 2,8) =-(qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 2,9) =-qp1*qpr1*cos(2*q2 + 2*q3);



            Yr( 2,10) =-qppr1*sin(q2 + q3);



            Yr( 2,11) =(qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 2,12) =-qppr1*cos(q2 + q3);



            Yr( 2,13) =qppr2 + qppr3;



            Yr( 2,14) =qppr1/2 + qppr2/2 - (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 - (qppr2*cos(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp3*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp4*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 2,15) =qppr1*sin(2*q2 + 2*q3 + 2*q4) + qppr2*sin(2*q2 + 2*q3 + 2*q4) - qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4) + qp1*qpr3*cos(2*q2 + 2*q3 + 2*q4) + qp2*qpr2*cos(2*q2 + 2*q3 + 2*q4) + qp3*qpr1*cos(2*q2 + 2*q3 + 2*q4) + qp1*qpr4*cos(2*q2 + 2*q3 + 2*q4) + qp2*qpr3*cos(2*q2 + 2*q3 + 2*q4) + qp3*qpr2*cos(2*q2 + 2*q3 + 2*q4) + qp4*qpr1*cos(2*q2 + 2*q3 + 2*q4) + qp2*qpr4*cos(2*q2 + 2*q3 + 2*q4) + qp4*qpr2*cos(2*q2 + 2*q3 + 2*q4);



            Yr( 2,16) =(qp1*qpr3*cos(q2 + q3 + q4))/2 - qppr4*sin(q2 + q3 + q4) - qppr3*sin(q2 + q3 + q4) + (qp3*qpr1*cos(q2 + q3 + q4))/2 + (qp1*qpr4*cos(q2 + q3 + q4))/2 + (qp4*qpr1*cos(q2 + q3 + q4))/2 - qp3*qpr3*cos(q2 + q3 + q4) - qp3*qpr4*cos(q2 + q3 + q4) - qp4*qpr3*cos(q2 + q3 + q4) - qp4*qpr4*cos(q2 + q3 + q4);



            Yr( 2,17) =qppr1/2 + qppr2/2 + (qppr1*cos(2*q2 + 2*q3 + 2*q4))/2 + (qppr2*cos(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr3*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qpr4*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qpr2*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 2,18) =qp3*qpr3*sin(q2 + q3 + q4) - qppr4*cos(q2 + q3 + q4) - (qp1*qpr3*sin(q2 + q3 + q4))/2 - (qp3*qpr1*sin(q2 + q3 + q4))/2 - (qp1*qpr4*sin(q2 + q3 + q4))/2 - (qp4*qpr1*sin(q2 + q3 + q4))/2 - qppr3*cos(q2 + q3 + q4) + qp3*qpr4*sin(q2 + q3 + q4) + qp4*qpr3*sin(q2 + q3 + q4) + qp4*qpr4*sin(q2 + q3 + q4);



            Yr( 2,19) =0;



            Yr( 2,20) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 2,21) =qppr2*sin(2*q5) + qppr3*sin(2*q5) + qppr4*sin(2*q5) + (qppr1*sin(q2 + q3 + q4 - 2*q5))/2 + (qppr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr5*cos(q2 + q3 + q4 - 2*q5))/2 + (qp1*qpr5*cos(q2 + q3 + q4 + 2*q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + qp2*qpr5*cos(2*q5) + qp5*qpr2*cos(2*q5) + qp3*qpr5*cos(2*q5) + qp5*qpr3*cos(2*q5) + qp4*qpr5*cos(2*q5) + qp5*qpr4*cos(2*q5);



            Yr( 2,22) =qppr5*sin(q5) - (qppr1*sin(q2 + q3 + q4 + q5))/2 + (qppr1*sin(q2 + q3 + q4 - q5))/2 + qp5*qpr5*cos(q5) - (qp1*qpr5*cos(q2 + q3 + q4 - q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*cos(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 2,23) =qppr2/2 + qppr3/2 + qppr4/2 + (qppr2*cos(2*q5))/2 + (qppr3*cos(2*q5))/2 + (qppr4*cos(2*q5))/2 - (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr5*sin(2*q5))/2 - (qp5*qpr2*sin(2*q5))/2 - (qp3*qpr5*sin(2*q5))/2 - (qp5*qpr3*sin(2*q5))/2 - (qp4*qpr5*sin(2*q5))/2 - (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 2,24) =qppr5*cos(q5) - (qppr1*cos(q2 + q3 + q4 + q5))/2 - (qppr1*cos(q2 + q3 + q4 - q5))/2 - qp5*qpr5*sin(q5) - (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 2,25) =(qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 - (qp1*qpr5*sin(q2 + q3 + q4))/2;



            Yr( 2,26) =qppr2/4 + qppr3/4 + qppr4/4 - (qppr2*cos(2*q5))/4 + (qppr2*cos(2*q6))/4 - (qppr3*cos(2*q5))/4 + (qppr3*cos(2*q6))/4 - (qppr4*cos(2*q5))/4 + (qppr4*cos(2*q6))/4 - (qppr2*cos(2*q5)*cos(2*q6))/4 - (qppr3*cos(2*q5)*cos(2*q6))/4 - (qppr4*cos(2*q5)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q5))/4 + (qp5*qpr2*sin(2*q5))/4 + (qp3*qpr5*sin(2*q5))/4 + (qp5*qpr3*sin(2*q5))/4 - (qp2*qpr6*sin(2*q6))/4 + (qp4*qpr5*sin(2*q5))/4 + (qp5*qpr4*sin(2*q5))/4 - (qp6*qpr2*sin(2*q6))/4 - (qp3*qpr6*sin(2*q6))/4 - (qp6*qpr3*sin(2*q6))/4 - (qp4*qpr6*sin(2*q6))/4 - (qp6*qpr4*sin(2*q6))/4 - (qppr5*sin(2*q6)*sin(q5))/2 - (qp5*qpr5*sin(2*q6)*cos(q5))/2 - (qp5*qpr6*cos(2*q6)*sin(q5))/2 - (qp6*qpr5*cos(2*q6)*sin(q5))/2 + (qp2*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr2*cos(2*q6)*sin(2*q5))/4 + (qp2*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr3*cos(2*q6)*sin(2*q5))/4 + (qp6*qpr2*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr4*cos(2*q6)*sin(2*q5))/4 + (qp6*qpr3*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp6*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2))/8 + (qp1*qpr5*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2))/8 - (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4))/8 - (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3))/8 - (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (3*qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6)*cos(q5))/2 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 - (qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2;



            Yr( 2,27) =(qppr2*cos(2*q5)*sin(2*q6))/2 - (qppr3*sin(2*q6))/2 - (qppr4*sin(2*q6))/2 - (qppr2*sin(2*q6))/2 + (qppr3*cos(2*q5)*sin(2*q6))/2 + (qppr4*cos(2*q5)*sin(2*q6))/2 - (qp2*qpr6*cos(2*q6))/2 - (qp6*qpr2*cos(2*q6))/2 - (qp3*qpr6*cos(2*q6))/2 - (qp6*qpr3*cos(2*q6))/2 - (qp4*qpr6*cos(2*q6))/2 - (qp6*qpr4*cos(2*q6))/2 - qppr5*cos(2*q6)*sin(q5) - qp5*qpr5*cos(2*q6)*cos(q5) + qp5*qpr6*sin(2*q6)*sin(q5) + qp6*qpr5*sin(2*q6)*sin(q5) + (qp2*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr2*cos(2*q5)*cos(2*q6))/2 + (qp3*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr3*cos(2*q5)*cos(2*q6))/2 + (qp4*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr4*cos(2*q5)*cos(2*q6))/2 - (qp2*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr2*sin(2*q5)*sin(2*q6))/2 - (qp3*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr3*sin(2*q5)*sin(2*q6))/2 - (qp4*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr4*sin(2*q5)*sin(2*q6))/2 + (qppr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 + (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 + (qp1*qpr5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (3*qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qppr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - (qppr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qppr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qppr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + qp1*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) + qp1*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) + qp1*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) + qp1*qpr6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp1*qpr6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp1*qpr6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 + (qp1*qpr5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) - qp1*qpr5*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - qp5*qpr1*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - qp1*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp6*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5);



            Yr( 2,28) =qppr5*cos(q5)*sin(q6) - qppr6*cos(q6)*sin(q5) - qppr2*sin(2*q5)*cos(q6) - qppr3*sin(2*q5)*cos(q6) - qppr4*sin(2*q5)*cos(q6) - qp5*qpr5*sin(q5)*sin(q6) + qp6*qpr6*sin(q5)*sin(q6) - qp2*qpr5*cos(2*q5)*cos(q6) - qp5*qpr2*cos(2*q5)*cos(q6) - qp3*qpr5*cos(2*q5)*cos(q6) - qp5*qpr3*cos(2*q5)*cos(q6) - qp4*qpr5*cos(2*q5)*cos(q6) - qp5*qpr4*cos(2*q5)*cos(q6) + (qp2*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr2*sin(2*q5)*sin(q6))/2 + (qp3*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr3*sin(2*q5)*sin(q6))/2 + (qp4*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr4*sin(2*q5)*sin(q6))/2 - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qppr1*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + qppr1*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) + qppr1*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qppr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qppr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qppr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qppr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp1*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp6*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp6*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp1*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 - qp1*qpr5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp5*qpr1*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + qp1*qpr5*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) + qp5*qpr1*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - qp5*qpr1*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) + qp1*qpr5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp1*qpr5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp1*qpr5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qp5*qpr1*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp5*qpr1*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp5*qpr1*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp1*qpr6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2;



            Yr( 2,29) =qppr2/4 + qppr3/4 + qppr4/4 - (qppr2*cos(2*q5))/4 - (qppr2*cos(2*q6))/4 - (qppr3*cos(2*q5))/4 - (qppr3*cos(2*q6))/4 - (qppr4*cos(2*q5))/4 - (qppr4*cos(2*q6))/4 + (qppr2*cos(2*q5)*cos(2*q6))/4 + (qppr3*cos(2*q5)*cos(2*q6))/4 + (qppr4*cos(2*q5)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q5))/4 + (qp5*qpr2*sin(2*q5))/4 + (qp3*qpr5*sin(2*q5))/4 + (qp5*qpr3*sin(2*q5))/4 + (qp2*qpr6*sin(2*q6))/4 + (qp4*qpr5*sin(2*q5))/4 + (qp5*qpr4*sin(2*q5))/4 + (qp6*qpr2*sin(2*q6))/4 + (qp3*qpr6*sin(2*q6))/4 + (qp6*qpr3*sin(2*q6))/4 + (qp4*qpr6*sin(2*q6))/4 + (qp6*qpr4*sin(2*q6))/4 + (qppr5*sin(2*q6)*sin(q5))/2 + (qp5*qpr5*sin(2*q6)*cos(q5))/2 + (qp5*qpr6*cos(2*q6)*sin(q5))/2 + (qp6*qpr5*cos(2*q6)*sin(q5))/2 - (qp2*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr2*cos(2*q6)*sin(2*q5))/4 - (qp2*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr3*cos(2*q6)*sin(2*q5))/4 - (qp6*qpr2*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr4*cos(2*q6)*sin(2*q5))/4 - (qp6*qpr3*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp6*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2))/8 + (qp1*qpr5*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2))/8 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4))/8 + (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3))/8 + (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (3*qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qppr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6)*cos(q5))/2 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2))/8 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 + (qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2;



            Yr( 2,30) =qppr5*cos(q5)*cos(q6) + qppr6*sin(q5)*sin(q6) + qppr2*sin(2*q5)*sin(q6) + qppr3*sin(2*q5)*sin(q6) + qppr4*sin(2*q5)*sin(q6) + qp2*qpr5*cos(2*q5)*sin(q6) + qp5*qpr2*cos(2*q5)*sin(q6) + (qp2*qpr6*sin(2*q5)*cos(q6))/2 + qp3*qpr5*cos(2*q5)*sin(q6) + qp5*qpr3*cos(2*q5)*sin(q6) + (qp6*qpr2*sin(2*q5)*cos(q6))/2 + (qp3*qpr6*sin(2*q5)*cos(q6))/2 + qp4*qpr5*cos(2*q5)*sin(q6) + qp5*qpr4*cos(2*q5)*sin(q6) + (qp6*qpr3*sin(2*q5)*cos(q6))/2 + (qp4*qpr6*sin(2*q5)*cos(q6))/2 + (qp6*qpr4*sin(2*q5)*cos(q6))/2 - qp5*qpr5*cos(q6)*sin(q5) + qp6*qpr6*cos(q6)*sin(q5) + qppr1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qppr1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qppr1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qppr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qppr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qppr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qppr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - (qp1*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp1*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp1*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp6*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp6*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp6*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp1*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 + qp1*qpr5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + qp1*qpr5*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) + qp5*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) - qp1*qpr5*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - qp1*qpr5*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - qp5*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) - qp5*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) + qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) + qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp1*qpr6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - qp1*qpr5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp1*qpr5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp1*qpr5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - (qp1*qpr6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 2,31) =qppr2/2 + qppr3/2 + qppr4/2 + (qppr2*cos(2*q5))/2 + (qppr3*cos(2*q5))/2 + (qppr4*cos(2*q5))/2 + qppr6*cos(q5) - (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr6*sin(q5))/2 - (qp6*qpr5*sin(q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr6*sin(q2 + q3 + q4 - q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp6*qpr1*sin(q2 + q3 + q4 - q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr5*sin(2*q5))/2 - (qp5*qpr2*sin(2*q5))/2 - (qp3*qpr5*sin(2*q5))/2 - (qp5*qpr3*sin(2*q5))/2 - (qp4*qpr5*sin(2*q5))/2 - (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr6*sin(q2 + q3 + q4 + q5))/4 + (qp6*qpr1*sin(q2 + q3 + q4 + q5))/4;



            Yr( 2,32) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(q2 + q3 + q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 2,33) =0;



            Yr( 2,34) =0;



            Yr( 2,35) =qppr2 + (qp1*qpr1*sin(2*q2))/2;



            Yr( 2,36) =0;



            Yr( 2,37) =qppr2 + (qp1*qpr1*sin(2*q2))/2;



            Yr( 2,38) =0;



            Yr( 2,39) =0;



            Yr( 2,40) =qppr2 + (qp1*qpr1*sin(2*q2))/2;



            Yr( 2,41) =qppr2 + (qp1*qpr1*sin(2*q2))/2;



            Yr( 2,42) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 2,43) =0;



            Yr( 2,44) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(q2 + q3 + q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 2,45) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 2,46) =qppr2 + (qp1*qpr1*sin(2*q2))/2;



            Yr( 2,47) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 2,48) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 2,49) =qppr2 + qppr3 + qppr4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 2,50) =0;



            Yr( 2,51) =0;



            Yr( 2,52) =0;



            Yr( 2,53) =0;



            Yr( 2,54) =0;



            Yr( 2,55) =-sin(q2 + q3 + q4)*cos(q1)*sin(q5);



            Yr( 2,56) =-sin(q2 + q3 + q4)*sin(q1)*sin(q5);



            Yr( 2,57) =sin(q2 + q3 + q4 - q5)/2 - sin(q2 + q3 + q4 + q5)/2;



            Yr( 2,58) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 2,59) =qppr1*cos(q5)*sin(q2) - qp1*qpr5*sin(q2)*sin(q5) - qp5*qpr1*sin(q2)*sin(q5) + 2*qppr2*cos(q3)*cos(q4)*sin(q5) + qppr3*cos(q3)*cos(q4)*sin(q5) + qppr4*cos(q3)*cos(q4)*sin(q5) + qppr5*cos(q3)*cos(q5)*sin(q4) + qppr5*cos(q4)*cos(q5)*sin(q3) - 2*qppr2*sin(q3)*sin(q4)*sin(q5) - qppr3*sin(q3)*sin(q4)*sin(q5) - qppr4*sin(q3)*sin(q4)*sin(q5) + qp2*qpr5*cos(q3)*cos(q4)*cos(q5) + qp5*qpr2*cos(q3)*cos(q4)*cos(q5) + qp3*qpr5*cos(q3)*cos(q4)*cos(q5) + qp5*qpr3*cos(q3)*cos(q4)*cos(q5) + qp4*qpr5*cos(q3)*cos(q4)*cos(q5) + qp5*qpr4*cos(q3)*cos(q4)*cos(q5) - qp2*qpr3*cos(q3)*sin(q4)*sin(q5) - qp2*qpr3*cos(q4)*sin(q3)*sin(q5) - qp3*qpr2*cos(q3)*sin(q4)*sin(q5) - qp3*qpr2*cos(q4)*sin(q3)*sin(q5) - qp2*qpr4*cos(q3)*sin(q4)*sin(q5) - qp2*qpr4*cos(q4)*sin(q3)*sin(q5) - qp3*qpr3*cos(q3)*sin(q4)*sin(q5) - qp3*qpr3*cos(q4)*sin(q3)*sin(q5) - qp4*qpr2*cos(q3)*sin(q4)*sin(q5) - qp4*qpr2*cos(q4)*sin(q3)*sin(q5) - qp2*qpr5*cos(q5)*sin(q3)*sin(q4) - qp3*qpr4*cos(q3)*sin(q4)*sin(q5) - qp3*qpr4*cos(q4)*sin(q3)*sin(q5) - qp4*qpr3*cos(q3)*sin(q4)*sin(q5) - qp4*qpr3*cos(q4)*sin(q3)*sin(q5) - qp5*qpr2*cos(q5)*sin(q3)*sin(q4) - qp3*qpr5*cos(q5)*sin(q3)*sin(q4) - qp4*qpr4*cos(q3)*sin(q4)*sin(q5) - qp4*qpr4*cos(q4)*sin(q3)*sin(q5) - qp5*qpr3*cos(q5)*sin(q3)*sin(q4) - qp4*qpr5*cos(q5)*sin(q3)*sin(q4) - qp5*qpr4*cos(q5)*sin(q3)*sin(q4) - qp5*qpr5*cos(q3)*sin(q4)*sin(q5) - qp5*qpr5*cos(q4)*sin(q3)*sin(q5) + qp1*qpr1*cos(2*q2)*cos(q3)*sin(q4)*sin(q5) + qp1*qpr1*cos(2*q2)*cos(q4)*sin(q3)*sin(q5) + qp1*qpr1*sin(2*q2)*cos(q3)*cos(q4)*sin(q5) - qp1*qpr1*sin(2*q2)*sin(q3)*sin(q4)*sin(q5);



            Yr( 2,60) =qppr2 + qppr3 + qppr4 - qppr2*cos(2*q5) - qppr3*cos(2*q5) - qppr4*cos(2*q5) + (qppr1*cos(q2 + q3 + q4 - 2*q5))/2 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - qp1*qpr5*sin(q2 + q3 + q4) - qp5*qpr1*sin(q2 + q3 + q4) + qp2*qpr5*sin(2*q5) + qp5*qpr2*sin(2*q5) + qp3*qpr5*sin(2*q5) + qp5*qpr3*sin(2*q5) + qp4*qpr5*sin(2*q5) + qp5*qpr4*sin(2*q5) + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 2,61) =2*qppr2*cos(q4)*sin(q5) + 2*qppr3*cos(q4)*sin(q5) + qppr4*cos(q4)*sin(q5) + qppr5*cos(q5)*sin(q4) + qp1*qpr1*sin(q4)*sin(q5) - qp2*qpr4*sin(q4)*sin(q5) - qp4*qpr2*sin(q4)*sin(q5) - qp3*qpr4*sin(q4)*sin(q5) - qp4*qpr3*sin(q4)*sin(q5) - qp4*qpr4*sin(q4)*sin(q5) - qp5*qpr5*sin(q4)*sin(q5) + qppr1*cos(q2)*cos(q5)*sin(q3) + qppr1*cos(q3)*cos(q5)*sin(q2) + qp2*qpr5*cos(q4)*cos(q5) + qp5*qpr2*cos(q4)*cos(q5) + qp3*qpr5*cos(q4)*cos(q5) + qp5*qpr3*cos(q4)*cos(q5) + qp4*qpr5*cos(q4)*cos(q5) + qp5*qpr4*cos(q4)*cos(q5) - 2*qp1*qpr1*pow(cos(q2),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*pow(cos(q3),2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q2)*sin(q3)*sin(q5) - qp1*qpr5*cos(q3)*sin(q2)*sin(q5) - qp5*qpr1*cos(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q3)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*cos(q2)*cos(q4)*sin(q2)*sin(q5) - 2*qp1*qpr1*cos(q3)*cos(q4)*sin(q3)*sin(q5) + 4*qp1*qpr1*cos(q2)*pow(cos(q3),2)*cos(q4)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 4*qp1*qpr1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5);



            Yr( 2,62) =(qppr1*cos(q2 + q3 + q4 + q5))/2 - qppr5*cos(q5) + (qppr1*cos(q2 + q3 + q4 - q5))/2 + qp5*qpr5*sin(q5) + (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 2,63) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 2,64) =(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 2,65) =0;



            Yr( 2,66) =0;



            Yr( 2,67) =-cos(q1)*sin(q2);



            Yr( 2,68) =0;



            Yr( 2,69) =-cos(q1)*sin(q2);



            Yr( 2,70) =0;



            Yr( 2,71) =0;



            Yr( 2,72) =-cos(q1)*sin(q2);



            Yr( 2,73) =-cos(q1)*sin(q2);



            Yr( 2,74) =-sin(q2 + q3)*cos(q1);



            Yr( 2,75) =0;



            Yr( 2,76) =-sin(q2 + q3 + q4)*cos(q1)*sin(q5);



            Yr( 2,77) =-sin(q2 + q3)*cos(q1);



            Yr( 2,78) =-cos(q1)*sin(q2);



            Yr( 2,79) =-sin(q2 + q3)*cos(q1);



            Yr( 2,80) =-sin(q2 + q3)*cos(q1);



            Yr( 2,81) =- cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2;



            Yr( 2,82) =0;



            Yr( 2,83) =0;



            Yr( 2,84) =0;



            Yr( 2,85) =0;



            Yr( 2,86) =0;



            Yr( 2,87) =0;



            Yr( 2,88) =0;



            Yr( 2,89) =-sin(q1)*sin(q2);



            Yr( 2,90) =0;



            Yr( 2,91) =-sin(q1)*sin(q2);



            Yr( 2,92) =0;



            Yr( 2,93) =0;



            Yr( 2,94) =-sin(q1)*sin(q2);



            Yr( 2,95) =-sin(q1)*sin(q2);



            Yr( 2,96) =-sin(q2 + q3)*sin(q1);



            Yr( 2,97) =0;



            Yr( 2,98) =-sin(q2 + q3 + q4)*sin(q1)*sin(q5);



            Yr( 2,99) =-sin(q2 + q3)*sin(q1);



            Yr( 2,100) =-sin(q1)*sin(q2);



            Yr( 2,101) =-sin(q2 + q3)*sin(q1);



            Yr( 2,102) =-sin(q2 + q3)*sin(q1);



            Yr( 2,103) =sin(q2 - q1 + q3 + q4)/2 - sin(q1 + q2 + q3 + q4)/2;



            Yr( 2,104) =0;



            Yr( 2,105) =0;



            Yr( 2,106) =0;



            Yr( 2,107) =0;



            Yr( 2,108) =0;



            Yr( 2,109) =-cos(q2);



            Yr( 2,110) =-cos(q2);



            Yr( 2,111) =-cos(q2);



            Yr( 2,112) =-cos(q2);



            Yr( 2,113) =-cos(q2 + q3);



            Yr( 2,114) =sin(q2 + q3 + q4 - q5)/2 - sin(q2 + q3 + q4 + q5)/2;



            Yr( 2,115) =-cos(q2 + q3);



            Yr( 2,116) =-cos(q2);



            Yr( 2,117) =-cos(q2 + q3);



            Yr( 2,118) =-cos(q2 + q3);



            Yr( 2,119) =sin(q2 + q3 + q4);



            Yr( 2,120) =qppr1*sin(q2);



            Yr( 2,121) =qppr1*sin(q2);



            Yr( 2,122) =0;



            Yr( 2,123) =qppr1*sin(q2);



            Yr( 2,124) =-qppr1*sin(q2);



            Yr( 2,125) =qppr1*sin(q2);



            Yr( 2,126) =qppr1*sin(q2 + q3);



            Yr( 2,127) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 2,128) =qppr1*sin(q2 + q3);



            Yr( 2,129) =2*qppr2*cos(q3) + qppr3*cos(q3) - qp2*qpr3*sin(q3) - qp3*qpr2*sin(q3) - qp3*qpr3*sin(q3) + qp1*qpr1*sin(2*q2 + q3);



            Yr( 2,130) =qppr1*sin(q2 + q3);



            Yr( 2,131) =qppr1*cos(q5)*sin(q2) - qp1*qpr5*sin(q2)*sin(q5) - qp5*qpr1*sin(q2)*sin(q5) + 2*qppr2*cos(q3)*cos(q4)*sin(q5) + qppr3*cos(q3)*cos(q4)*sin(q5) + qppr4*cos(q3)*cos(q4)*sin(q5) + qppr5*cos(q3)*cos(q5)*sin(q4) + qppr5*cos(q4)*cos(q5)*sin(q3) - 2*qppr2*sin(q3)*sin(q4)*sin(q5) - qppr3*sin(q3)*sin(q4)*sin(q5) - qppr4*sin(q3)*sin(q4)*sin(q5) + qp2*qpr5*cos(q3)*cos(q4)*cos(q5) + qp5*qpr2*cos(q3)*cos(q4)*cos(q5) + qp3*qpr5*cos(q3)*cos(q4)*cos(q5) + qp5*qpr3*cos(q3)*cos(q4)*cos(q5) + qp4*qpr5*cos(q3)*cos(q4)*cos(q5) + qp5*qpr4*cos(q3)*cos(q4)*cos(q5) - qp2*qpr3*cos(q3)*sin(q4)*sin(q5) - qp2*qpr3*cos(q4)*sin(q3)*sin(q5) - qp3*qpr2*cos(q3)*sin(q4)*sin(q5) - qp3*qpr2*cos(q4)*sin(q3)*sin(q5) - qp2*qpr4*cos(q3)*sin(q4)*sin(q5) - qp2*qpr4*cos(q4)*sin(q3)*sin(q5) - qp3*qpr3*cos(q3)*sin(q4)*sin(q5) - qp3*qpr3*cos(q4)*sin(q3)*sin(q5) - qp4*qpr2*cos(q3)*sin(q4)*sin(q5) - qp4*qpr2*cos(q4)*sin(q3)*sin(q5) - qp2*qpr5*cos(q5)*sin(q3)*sin(q4) - qp3*qpr4*cos(q3)*sin(q4)*sin(q5) - qp3*qpr4*cos(q4)*sin(q3)*sin(q5) - qp4*qpr3*cos(q3)*sin(q4)*sin(q5) - qp4*qpr3*cos(q4)*sin(q3)*sin(q5) - qp5*qpr2*cos(q5)*sin(q3)*sin(q4) - qp3*qpr5*cos(q5)*sin(q3)*sin(q4) - qp4*qpr4*cos(q3)*sin(q4)*sin(q5) - qp4*qpr4*cos(q4)*sin(q3)*sin(q5) - qp5*qpr3*cos(q5)*sin(q3)*sin(q4) - qp4*qpr5*cos(q5)*sin(q3)*sin(q4) - qp5*qpr4*cos(q5)*sin(q3)*sin(q4) - qp5*qpr5*cos(q3)*sin(q4)*sin(q5) - qp5*qpr5*cos(q4)*sin(q3)*sin(q5) + qp1*qpr1*cos(2*q2)*cos(q3)*sin(q4)*sin(q5) + qp1*qpr1*cos(2*q2)*cos(q4)*sin(q3)*sin(q5) + qp1*qpr1*sin(2*q2)*cos(q3)*cos(q4)*sin(q5) - qp1*qpr1*sin(2*q2)*sin(q3)*sin(q4)*sin(q5);



            Yr( 2,132) =2*qppr2*cos(q3) + qppr3*cos(q3) - qp2*qpr3*sin(q3) - qp3*qpr2*sin(q3) - qp3*qpr3*sin(q3) + qp1*qpr1*sin(2*q2 + q3);



            Yr( 2,133) =2*qppr2*cos(q3) + qppr3*cos(q3) - qp2*qpr3*sin(q3) - qp3*qpr2*sin(q3) - qp3*qpr3*sin(q3) + qp1*qpr1*sin(2*q2 + q3);



            Yr( 2,134) =qppr1*sin(q2 + q3);



            Yr( 2,135) =2*qppr2*cos(q4)*sin(q5) + 2*qppr3*cos(q4)*sin(q5) + qppr4*cos(q4)*sin(q5) + qppr5*cos(q5)*sin(q4) + qp1*qpr1*sin(q4)*sin(q5) - qp2*qpr4*sin(q4)*sin(q5) - qp4*qpr2*sin(q4)*sin(q5) - qp3*qpr4*sin(q4)*sin(q5) - qp4*qpr3*sin(q4)*sin(q5) - qp4*qpr4*sin(q4)*sin(q5) - qp5*qpr5*sin(q4)*sin(q5) + qppr1*cos(q2)*cos(q5)*sin(q3) + qppr1*cos(q3)*cos(q5)*sin(q2) + qp2*qpr5*cos(q4)*cos(q5) + qp5*qpr2*cos(q4)*cos(q5) + qp3*qpr5*cos(q4)*cos(q5) + qp5*qpr3*cos(q4)*cos(q5) + qp4*qpr5*cos(q4)*cos(q5) + qp5*qpr4*cos(q4)*cos(q5) - 2*qp1*qpr1*pow(cos(q2),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*pow(cos(q3),2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q2)*sin(q3)*sin(q5) - qp1*qpr5*cos(q3)*sin(q2)*sin(q5) - qp5*qpr1*cos(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q3)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*cos(q2)*cos(q4)*sin(q2)*sin(q5) - 2*qp1*qpr1*cos(q3)*cos(q4)*sin(q3)*sin(q5) + 4*qp1*qpr1*cos(q2)*pow(cos(q3),2)*cos(q4)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 4*qp1*qpr1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5);



            Yr( 2,136) =2*qppr2*cos(q3) + qppr3*cos(q3) - qp2*qpr3*sin(q3) - qp3*qpr2*sin(q3) - qp3*qpr3*sin(q3) + qp1*qpr1*sin(2*q2 + q3);



            Yr( 2,137) =qppr1*cos(q2 + q3 + q4);



            Yr( 2,138) =-qppr1*sin(q2 + q3);



            Yr( 2,139) =qppr1*sin(q2);



            Yr( 2,140) =0;



            Yr( 2,141) =0;



            Yr( 2,142) =qp1*qpr1*cos(2*q2 + q3 + q4) - qppr3*sin(q3 + q4) - qppr4*sin(q3 + q4) - 2*qppr2*sin(q3 + q4) - qp2*qpr3*cos(q3 + q4) - qp3*qpr2*cos(q3 + q4) - qp2*qpr4*cos(q3 + q4) - qp3*qpr3*cos(q3 + q4) - qp4*qpr2*cos(q3 + q4) - qp3*qpr4*cos(q3 + q4) - qp4*qpr3*cos(q3 + q4) - qp4*qpr4*cos(q3 + q4);



            Yr( 2,143) =0;



            Yr( 2,144) =0;



            Yr( 2,145) =qppr1*sin(q2);



            Yr( 2,146) =-qppr1*sin(q2);



            Yr( 2,147) =(qppr1*cos(q2 + q3 + q4 + q5))/2 - qppr5*cos(q5) + (qppr1*cos(q2 + q3 + q4 - q5))/2 + qp5*qpr5*sin(q5) + (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 2,148) =0;



            Yr( 2,149) =qppr1*sin(q2);



            Yr( 2,150) =-qppr1*sin(q2);



            Yr( 2,151) =qp1*qpr1*cos(2*q2 + 2*q3 + q4) - 2*qppr3*sin(q4) - qppr4*sin(q4) - qp2*qpr4*cos(q4) - qp4*qpr2*cos(q4) - qp3*qpr4*cos(q4) - qp4*qpr3*cos(q4) - qp4*qpr4*cos(q4) - 2*qppr2*sin(q4);



            Yr( 2,152) =-qppr1*sin(q2);



            Yr( 2,153) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 2,154) =qppr1*sin(q2 + q3);



            Yr( 2,155) =-qppr1*sin(q2 + q3);



            Yr( 2,156) =(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 2,157) =qppr1*sin(q2 + q3);



            Yr( 2,158) =-qppr1*sin(q2 + q3);



            Yr( 2,159) =-qppr1*sin(q2 + q3);



            Yr( 2,160) =qppr1*cos(q2 + q3 + q4);



            Yr( 2,161) =-qppr1*cos(q2 + q3 + q4);



            Yr( 2,162) =0;



            Yr( 2,163) =0;



            Yr( 3,1) =0;



            Yr( 3,2) =0;



            Yr( 3,3) =0;



            Yr( 3,4) =0;



            Yr( 3,5) =0;



            Yr( 3,6) =0;



            Yr( 3,7) =0;



            Yr( 3,8) =-(qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 3,9) =-qp1*qpr1*cos(2*q2 + 2*q3);



            Yr( 3,10) =-qppr1*sin(q2 + q3);



            Yr( 3,11) =(qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 3,12) =-qppr1*cos(q2 + q3);



            Yr( 3,13) =qppr2 + qppr3;



            Yr( 3,14) =-(sin(2*q2 + 2*q3 + 2*q4)*(qp1 + qp2)*(qpr1 + qpr2))/2;



            Yr( 3,15) =-cos(2*q2 + 2*q3 + 2*q4)*(qp1 + qp2)*(qpr1 + qpr2);



            Yr( 3,16) =- qppr1*sin(q2 + q3 + q4) - qppr2*sin(q2 + q3 + q4) - (qp1*qpr2*cos(q2 + q3 + q4))/2 - (qp2*qpr1*cos(q2 + q3 + q4))/2 - qp2*qpr2*cos(q2 + q3 + q4);



            Yr( 3,17) =(sin(2*q2 + 2*q3 + 2*q4)*(qp1 + qp2)*(qpr1 + qpr2))/2;



            Yr( 3,18) =(qp1*qpr2*sin(q2 + q3 + q4))/2 - qppr2*cos(q2 + q3 + q4) - qppr1*cos(q2 + q3 + q4) + (qp2*qpr1*sin(q2 + q3 + q4))/2 + qp2*qpr2*sin(q2 + q3 + q4);



            Yr( 3,19) =qppr3 + qppr4;



            Yr( 3,20) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 3,21) =qppr2*sin(2*q5) + qppr3*sin(2*q5) + qppr4*sin(2*q5) + (qppr1*sin(q2 + q3 + q4 - 2*q5))/2 + (qppr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr5*cos(q2 + q3 + q4 - 2*q5))/2 + (qp1*qpr5*cos(q2 + q3 + q4 + 2*q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + qp2*qpr5*cos(2*q5) + qp5*qpr2*cos(2*q5) + qp3*qpr5*cos(2*q5) + qp5*qpr3*cos(2*q5) + qp4*qpr5*cos(2*q5) + qp5*qpr4*cos(2*q5);



            Yr( 3,22) =qppr5*sin(q5) - (qppr1*sin(q2 + q3 + q4 + q5))/2 + (qppr1*sin(q2 + q3 + q4 - q5))/2 + qp5*qpr5*cos(q5) - (qp1*qpr5*cos(q2 + q3 + q4 - q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*cos(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 3,23) =qppr2/2 + qppr3/2 + qppr4/2 + (qppr2*cos(2*q5))/2 + (qppr3*cos(2*q5))/2 + (qppr4*cos(2*q5))/2 - (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr5*sin(2*q5))/2 - (qp5*qpr2*sin(2*q5))/2 - (qp3*qpr5*sin(2*q5))/2 - (qp5*qpr3*sin(2*q5))/2 - (qp4*qpr5*sin(2*q5))/2 - (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 3,24) =qppr5*cos(q5) - (qppr1*cos(q2 + q3 + q4 + q5))/2 - (qppr1*cos(q2 + q3 + q4 - q5))/2 - qp5*qpr5*sin(q5) - (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 3,25) =(qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 - (qp1*qpr5*sin(q2 + q3 + q4))/2;



            Yr( 3,26) =qppr2/4 + qppr3/4 + qppr4/4 - (qppr2*cos(2*q5))/4 + (qppr2*cos(2*q6))/4 - (qppr3*cos(2*q5))/4 + (qppr3*cos(2*q6))/4 - (qppr4*cos(2*q5))/4 + (qppr4*cos(2*q6))/4 - (qppr2*cos(2*q5)*cos(2*q6))/4 - (qppr3*cos(2*q5)*cos(2*q6))/4 - (qppr4*cos(2*q5)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q5))/4 + (qp5*qpr2*sin(2*q5))/4 + (qp3*qpr5*sin(2*q5))/4 + (qp5*qpr3*sin(2*q5))/4 - (qp2*qpr6*sin(2*q6))/4 + (qp4*qpr5*sin(2*q5))/4 + (qp5*qpr4*sin(2*q5))/4 - (qp6*qpr2*sin(2*q6))/4 - (qp3*qpr6*sin(2*q6))/4 - (qp6*qpr3*sin(2*q6))/4 - (qp4*qpr6*sin(2*q6))/4 - (qp6*qpr4*sin(2*q6))/4 - (qppr5*sin(2*q6)*sin(q5))/2 - (qp5*qpr5*sin(2*q6)*cos(q5))/2 - (qp5*qpr6*cos(2*q6)*sin(q5))/2 - (qp6*qpr5*cos(2*q6)*sin(q5))/2 + (qp2*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr2*cos(2*q6)*sin(2*q5))/4 + (qp2*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr3*cos(2*q6)*sin(2*q5))/4 + (qp6*qpr2*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr4*cos(2*q6)*sin(2*q5))/4 + (qp6*qpr3*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp6*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2))/8 + (qp1*qpr5*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2))/8 - (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4))/8 - (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3))/8 - (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (3*qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6)*cos(q5))/2 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 - (qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2;



            Yr( 3,27) =(qppr2*cos(2*q5)*sin(2*q6))/2 - (qppr3*sin(2*q6))/2 - (qppr4*sin(2*q6))/2 - (qppr2*sin(2*q6))/2 + (qppr3*cos(2*q5)*sin(2*q6))/2 + (qppr4*cos(2*q5)*sin(2*q6))/2 - (qp2*qpr6*cos(2*q6))/2 - (qp6*qpr2*cos(2*q6))/2 - (qp3*qpr6*cos(2*q6))/2 - (qp6*qpr3*cos(2*q6))/2 - (qp4*qpr6*cos(2*q6))/2 - (qp6*qpr4*cos(2*q6))/2 - qppr5*cos(2*q6)*sin(q5) - qp5*qpr5*cos(2*q6)*cos(q5) + qp5*qpr6*sin(2*q6)*sin(q5) + qp6*qpr5*sin(2*q6)*sin(q5) + (qp2*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr2*cos(2*q5)*cos(2*q6))/2 + (qp3*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr3*cos(2*q5)*cos(2*q6))/2 + (qp4*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr4*cos(2*q5)*cos(2*q6))/2 - (qp2*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr2*sin(2*q5)*sin(2*q6))/2 - (qp3*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr3*sin(2*q5)*sin(2*q6))/2 - (qp4*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr4*sin(2*q5)*sin(2*q6))/2 + (qppr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 + (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 + (qp1*qpr5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (3*qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qppr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - (qppr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qppr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qppr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + qp1*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) + qp1*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) + qp1*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) + qp1*qpr6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp1*qpr6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp1*qpr6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 + (qp1*qpr5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) - qp1*qpr5*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - qp5*qpr1*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - qp1*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp6*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5);



            Yr( 3,28) =qppr5*cos(q5)*sin(q6) - qppr6*cos(q6)*sin(q5) - qppr2*sin(2*q5)*cos(q6) - qppr3*sin(2*q5)*cos(q6) - qppr4*sin(2*q5)*cos(q6) - qp5*qpr5*sin(q5)*sin(q6) + qp6*qpr6*sin(q5)*sin(q6) - qp2*qpr5*cos(2*q5)*cos(q6) - qp5*qpr2*cos(2*q5)*cos(q6) - qp3*qpr5*cos(2*q5)*cos(q6) - qp5*qpr3*cos(2*q5)*cos(q6) - qp4*qpr5*cos(2*q5)*cos(q6) - qp5*qpr4*cos(2*q5)*cos(q6) + (qp2*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr2*sin(2*q5)*sin(q6))/2 + (qp3*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr3*sin(2*q5)*sin(q6))/2 + (qp4*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr4*sin(2*q5)*sin(q6))/2 - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qppr1*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + qppr1*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) + qppr1*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qppr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qppr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qppr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qppr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp1*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp6*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp6*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp1*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 - qp1*qpr5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp5*qpr1*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + qp1*qpr5*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) + qp5*qpr1*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - qp5*qpr1*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) + qp1*qpr5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp1*qpr5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp1*qpr5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qp5*qpr1*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp5*qpr1*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp5*qpr1*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp1*qpr6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2;



            Yr( 3,29) =qppr2/4 + qppr3/4 + qppr4/4 - (qppr2*cos(2*q5))/4 - (qppr2*cos(2*q6))/4 - (qppr3*cos(2*q5))/4 - (qppr3*cos(2*q6))/4 - (qppr4*cos(2*q5))/4 - (qppr4*cos(2*q6))/4 + (qppr2*cos(2*q5)*cos(2*q6))/4 + (qppr3*cos(2*q5)*cos(2*q6))/4 + (qppr4*cos(2*q5)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q5))/4 + (qp5*qpr2*sin(2*q5))/4 + (qp3*qpr5*sin(2*q5))/4 + (qp5*qpr3*sin(2*q5))/4 + (qp2*qpr6*sin(2*q6))/4 + (qp4*qpr5*sin(2*q5))/4 + (qp5*qpr4*sin(2*q5))/4 + (qp6*qpr2*sin(2*q6))/4 + (qp3*qpr6*sin(2*q6))/4 + (qp6*qpr3*sin(2*q6))/4 + (qp4*qpr6*sin(2*q6))/4 + (qp6*qpr4*sin(2*q6))/4 + (qppr5*sin(2*q6)*sin(q5))/2 + (qp5*qpr5*sin(2*q6)*cos(q5))/2 + (qp5*qpr6*cos(2*q6)*sin(q5))/2 + (qp6*qpr5*cos(2*q6)*sin(q5))/2 - (qp2*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr2*cos(2*q6)*sin(2*q5))/4 - (qp2*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr3*cos(2*q6)*sin(2*q5))/4 - (qp6*qpr2*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr4*cos(2*q6)*sin(2*q5))/4 - (qp6*qpr3*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp6*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2))/8 + (qp1*qpr5*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2))/8 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4))/8 + (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3))/8 + (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (3*qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qppr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6)*cos(q5))/2 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2))/8 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 + (qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2;



            Yr( 3,30) =qppr5*cos(q5)*cos(q6) + qppr6*sin(q5)*sin(q6) + qppr2*sin(2*q5)*sin(q6) + qppr3*sin(2*q5)*sin(q6) + qppr4*sin(2*q5)*sin(q6) + qp2*qpr5*cos(2*q5)*sin(q6) + qp5*qpr2*cos(2*q5)*sin(q6) + (qp2*qpr6*sin(2*q5)*cos(q6))/2 + qp3*qpr5*cos(2*q5)*sin(q6) + qp5*qpr3*cos(2*q5)*sin(q6) + (qp6*qpr2*sin(2*q5)*cos(q6))/2 + (qp3*qpr6*sin(2*q5)*cos(q6))/2 + qp4*qpr5*cos(2*q5)*sin(q6) + qp5*qpr4*cos(2*q5)*sin(q6) + (qp6*qpr3*sin(2*q5)*cos(q6))/2 + (qp4*qpr6*sin(2*q5)*cos(q6))/2 + (qp6*qpr4*sin(2*q5)*cos(q6))/2 - qp5*qpr5*cos(q6)*sin(q5) + qp6*qpr6*cos(q6)*sin(q5) + qppr1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qppr1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qppr1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qppr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qppr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qppr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qppr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - (qp1*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp1*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp1*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp6*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp6*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp6*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp1*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 + qp1*qpr5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + qp1*qpr5*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) + qp5*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) - qp1*qpr5*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - qp1*qpr5*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - qp5*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) - qp5*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) + qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) + qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp1*qpr6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - qp1*qpr5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp1*qpr5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp1*qpr5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - (qp1*qpr6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 3,31) =qppr2/2 + qppr3/2 + qppr4/2 + (qppr2*cos(2*q5))/2 + (qppr3*cos(2*q5))/2 + (qppr4*cos(2*q5))/2 + qppr6*cos(q5) - (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr6*sin(q5))/2 - (qp6*qpr5*sin(q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr6*sin(q2 + q3 + q4 - q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp6*qpr1*sin(q2 + q3 + q4 - q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr5*sin(2*q5))/2 - (qp5*qpr2*sin(2*q5))/2 - (qp3*qpr5*sin(2*q5))/2 - (qp5*qpr3*sin(2*q5))/2 - (qp4*qpr5*sin(2*q5))/2 - (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr6*sin(q2 + q3 + q4 + q5))/4 + (qp6*qpr1*sin(q2 + q3 + q4 + q5))/4;



            Yr( 3,32) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(q2 + q3 + q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 3,33) =0;



            Yr( 3,34) =0;



            Yr( 3,35) =0;



            Yr( 3,36) =0;



            Yr( 3,37) =0;



            Yr( 3,38) =0;



            Yr( 3,39) =0;



            Yr( 3,40) =0;



            Yr( 3,41) =0;



            Yr( 3,42) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 3,43) =0;



            Yr( 3,44) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(q2 + q3 + q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 3,45) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 3,46) =0;



            Yr( 3,47) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 3,48) =qppr2 + qppr3 + (qp1*qpr1*sin(2*q2 + 2*q3))/2;



            Yr( 3,49) =qppr2 + qppr3 + qppr4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 3,50) =0;



            Yr( 3,51) =0;



            Yr( 3,52) =0;



            Yr( 3,53) =0;



            Yr( 3,54) =0;



            Yr( 3,55) =-sin(q2 + q3 + q4)*cos(q1)*sin(q5);



            Yr( 3,56) =-sin(q2 + q3 + q4)*sin(q1)*sin(q5);



            Yr( 3,57) =sin(q2 + q3 + q4 - q5)/2 - sin(q2 + q3 + q4 + q5)/2;



            Yr( 3,58) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 3,59) =sin(q5)*(qppr2*cos(q3 + q4) + (qp1*qpr1*sin(2*q2 + q3 + q4))/2 + (qp1*qpr1*sin(q3 + q4))/2 + qp2*qpr2*sin(q3 + q4));



            Yr( 3,60) =qppr2 + qppr3 + qppr4 - qppr2*cos(2*q5) - qppr3*cos(2*q5) - qppr4*cos(2*q5) + (qppr1*cos(q2 + q3 + q4 - 2*q5))/2 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - qp1*qpr5*sin(q2 + q3 + q4) - qp5*qpr1*sin(q2 + q3 + q4) + qp2*qpr5*sin(2*q5) + qp5*qpr2*sin(2*q5) + qp3*qpr5*sin(2*q5) + qp5*qpr3*sin(2*q5) + qp4*qpr5*sin(2*q5) + qp5*qpr4*sin(2*q5) + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 3,61) =2*qppr2*cos(q4)*sin(q5) + 2*qppr3*cos(q4)*sin(q5) + qppr4*cos(q4)*sin(q5) + qppr5*cos(q5)*sin(q4) + qp1*qpr1*sin(q4)*sin(q5) - qp2*qpr4*sin(q4)*sin(q5) - qp4*qpr2*sin(q4)*sin(q5) - qp3*qpr4*sin(q4)*sin(q5) - qp4*qpr3*sin(q4)*sin(q5) - qp4*qpr4*sin(q4)*sin(q5) - qp5*qpr5*sin(q4)*sin(q5) + qppr1*cos(q2)*cos(q5)*sin(q3) + qppr1*cos(q3)*cos(q5)*sin(q2) + qp2*qpr5*cos(q4)*cos(q5) + qp5*qpr2*cos(q4)*cos(q5) + qp3*qpr5*cos(q4)*cos(q5) + qp5*qpr3*cos(q4)*cos(q5) + qp4*qpr5*cos(q4)*cos(q5) + qp5*qpr4*cos(q4)*cos(q5) - 2*qp1*qpr1*pow(cos(q2),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*pow(cos(q3),2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q2)*sin(q3)*sin(q5) - qp1*qpr5*cos(q3)*sin(q2)*sin(q5) - qp5*qpr1*cos(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q3)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*cos(q2)*cos(q4)*sin(q2)*sin(q5) - 2*qp1*qpr1*cos(q3)*cos(q4)*sin(q3)*sin(q5) + 4*qp1*qpr1*cos(q2)*pow(cos(q3),2)*cos(q4)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 4*qp1*qpr1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5);



            Yr( 3,62) =(qppr1*cos(q2 + q3 + q4 + q5))/2 - qppr5*cos(q5) + (qppr1*cos(q2 + q3 + q4 - q5))/2 + qp5*qpr5*sin(q5) + (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 3,63) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 3,64) =(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 3,65) =0;



            Yr( 3,66) =0;



            Yr( 3,67) =0;



            Yr( 3,68) =0;



            Yr( 3,69) =0;



            Yr( 3,70) =0;



            Yr( 3,71) =0;



            Yr( 3,72) =0;



            Yr( 3,73) =0;



            Yr( 3,74) =-sin(q2 + q3)*cos(q1);



            Yr( 3,75) =0;



            Yr( 3,76) =-sin(q2 + q3 + q4)*cos(q1)*sin(q5);



            Yr( 3,77) =-sin(q2 + q3)*cos(q1);



            Yr( 3,78) =0;



            Yr( 3,79) =-sin(q2 + q3)*cos(q1);



            Yr( 3,80) =-sin(q2 + q3)*cos(q1);



            Yr( 3,81) =- cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2;



            Yr( 3,82) =0;



            Yr( 3,83) =0;



            Yr( 3,84) =0;



            Yr( 3,85) =0;



            Yr( 3,86) =0;



            Yr( 3,87) =0;



            Yr( 3,88) =0;



            Yr( 3,89) =0;



            Yr( 3,90) =0;



            Yr( 3,91) =0;



            Yr( 3,92) =0;



            Yr( 3,93) =0;



            Yr( 3,94) =0;



            Yr( 3,95) =0;



            Yr( 3,96) =-sin(q2 + q3)*sin(q1);



            Yr( 3,97) =0;



            Yr( 3,98) =-sin(q2 + q3 + q4)*sin(q1)*sin(q5);



            Yr( 3,99) =-sin(q2 + q3)*sin(q1);



            Yr( 3,100) =0;



            Yr( 3,101) =-sin(q2 + q3)*sin(q1);



            Yr( 3,102) =-sin(q2 + q3)*sin(q1);



            Yr( 3,103) =sin(q2 - q1 + q3 + q4)/2 - sin(q1 + q2 + q3 + q4)/2;



            Yr( 3,104) =0;



            Yr( 3,105) =0;



            Yr( 3,106) =0;



            Yr( 3,107) =0;



            Yr( 3,108) =0;



            Yr( 3,109) =0;



            Yr( 3,110) =0;



            Yr( 3,111) =0;



            Yr( 3,112) =0;



            Yr( 3,113) =-cos(q2 + q3);



            Yr( 3,114) =sin(q2 + q3 + q4 - q5)/2 - sin(q2 + q3 + q4 + q5)/2;



            Yr( 3,115) =-cos(q2 + q3);



            Yr( 3,116) =0;



            Yr( 3,117) =-cos(q2 + q3);



            Yr( 3,118) =-cos(q2 + q3);



            Yr( 3,119) =sin(q2 + q3 + q4);



            Yr( 3,120) =0;



            Yr( 3,121) =0;



            Yr( 3,122) =0;



            Yr( 3,123) =0;



            Yr( 3,124) =0;



            Yr( 3,125) =0;



            Yr( 3,126) =qppr1*sin(q2 + q3);



            Yr( 3,127) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 3,128) =qppr1*sin(q2 + q3);



            Yr( 3,129) =qppr2*cos(q3) + (qp1*qpr1*sin(q3))/2 + qp2*qpr2*sin(q3) + (qp1*qpr1*sin(2*q2 + q3))/2;



            Yr( 3,130) =qppr1*sin(q2 + q3);



            Yr( 3,131) =sin(q5)*(qppr2*cos(q3 + q4) + (qp1*qpr1*sin(2*q2 + q3 + q4))/2 + (qp1*qpr1*sin(q3 + q4))/2 + qp2*qpr2*sin(q3 + q4));



            Yr( 3,132) =qppr2*cos(q3) + (qp1*qpr1*sin(q3))/2 + qp2*qpr2*sin(q3) + (qp1*qpr1*sin(2*q2 + q3))/2;



            Yr( 3,133) =qppr2*cos(q3) + (qp1*qpr1*sin(q3))/2 + qp2*qpr2*sin(q3) + (qp1*qpr1*sin(2*q2 + q3))/2;



            Yr( 3,134) =qppr1*sin(q2 + q3);



            Yr( 3,135) =2*qppr2*cos(q4)*sin(q5) + 2*qppr3*cos(q4)*sin(q5) + qppr4*cos(q4)*sin(q5) + qppr5*cos(q5)*sin(q4) + qp1*qpr1*sin(q4)*sin(q5) - qp2*qpr4*sin(q4)*sin(q5) - qp4*qpr2*sin(q4)*sin(q5) - qp3*qpr4*sin(q4)*sin(q5) - qp4*qpr3*sin(q4)*sin(q5) - qp4*qpr4*sin(q4)*sin(q5) - qp5*qpr5*sin(q4)*sin(q5) + qppr1*cos(q2)*cos(q5)*sin(q3) + qppr1*cos(q3)*cos(q5)*sin(q2) + qp2*qpr5*cos(q4)*cos(q5) + qp5*qpr2*cos(q4)*cos(q5) + qp3*qpr5*cos(q4)*cos(q5) + qp5*qpr3*cos(q4)*cos(q5) + qp4*qpr5*cos(q4)*cos(q5) + qp5*qpr4*cos(q4)*cos(q5) - 2*qp1*qpr1*pow(cos(q2),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*pow(cos(q3),2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q2)*sin(q3)*sin(q5) - qp1*qpr5*cos(q3)*sin(q2)*sin(q5) - qp5*qpr1*cos(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q3)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*sin(q4)*sin(q5) - 2*qp1*qpr1*cos(q2)*cos(q4)*sin(q2)*sin(q5) - 2*qp1*qpr1*cos(q3)*cos(q4)*sin(q3)*sin(q5) + 4*qp1*qpr1*cos(q2)*pow(cos(q3),2)*cos(q4)*sin(q2)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 4*qp1*qpr1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5);



            Yr( 3,136) =qppr2*cos(q3) + (qp1*qpr1*sin(q3))/2 + qp2*qpr2*sin(q3) + (qp1*qpr1*sin(2*q2 + q3))/2;



            Yr( 3,137) =qppr1*cos(q2 + q3 + q4);



            Yr( 3,138) =-qppr1*sin(q2 + q3);



            Yr( 3,139) =0;



            Yr( 3,140) =0;



            Yr( 3,141) =0;



            Yr( 3,142) =(qp1*qpr1*cos(2*q2 + q3 + q4))/2 - qppr2*sin(q3 + q4) + (qp1*qpr1*cos(q3 + q4))/2 + qp2*qpr2*cos(q3 + q4);



            Yr( 3,143) =0;



            Yr( 3,144) =0;



            Yr( 3,145) =0;



            Yr( 3,146) =0;



            Yr( 3,147) =(qppr1*cos(q2 + q3 + q4 + q5))/2 - qppr5*cos(q5) + (qppr1*cos(q2 + q3 + q4 - q5))/2 + qp5*qpr5*sin(q5) + (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 3,148) =0;



            Yr( 3,149) =0;



            Yr( 3,150) =0;



            Yr( 3,151) =qp1*qpr1*cos(2*q2 + 2*q3 + q4) - 2*qppr3*sin(q4) - qppr4*sin(q4) - qp2*qpr4*cos(q4) - qp4*qpr2*cos(q4) - qp3*qpr4*cos(q4) - qp4*qpr3*cos(q4) - qp4*qpr4*cos(q4) - 2*qppr2*sin(q4);



            Yr( 3,152) =0;



            Yr( 3,153) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 3,154) =qppr1*sin(q2 + q3);



            Yr( 3,155) =-qppr1*sin(q2 + q3);



            Yr( 3,156) =(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 3,157) =qppr1*sin(q2 + q3);



            Yr( 3,158) =-qppr1*sin(q2 + q3);



            Yr( 3,159) =-qppr1*sin(q2 + q3);



            Yr( 3,160) =qppr1*cos(q2 + q3 + q4);



            Yr( 3,161) =-qppr1*cos(q2 + q3 + q4);



            Yr( 3,162) =0;



            Yr( 3,163) =0;



            Yr( 4,1) =0;



            Yr( 4,2) =0;



            Yr( 4,3) =0;



            Yr( 4,4) =0;



            Yr( 4,5) =0;



            Yr( 4,6) =0;



            Yr( 4,7) =0;



            Yr( 4,8) =0;



            Yr( 4,9) =0;



            Yr( 4,10) =0;



            Yr( 4,11) =0;



            Yr( 4,12) =0;



            Yr( 4,13) =0;



            Yr( 4,14) =-(sin(2*q2 + 2*q3 + 2*q4)*(qp1 + qp2)*(qpr1 + qpr2))/2;



            Yr( 4,15) =-cos(2*q2 + 2*q3 + 2*q4)*(qp1 + qp2)*(qpr1 + qpr2);



            Yr( 4,16) =- qppr1*sin(q2 + q3 + q4) - qppr2*sin(q2 + q3 + q4) - (qp1*qpr2*cos(q2 + q3 + q4))/2 - (qp2*qpr1*cos(q2 + q3 + q4))/2 - qp2*qpr2*cos(q2 + q3 + q4);



            Yr( 4,17) =(sin(2*q2 + 2*q3 + 2*q4)*(qp1 + qp2)*(qpr1 + qpr2))/2;



            Yr( 4,18) =(qp1*qpr2*sin(q2 + q3 + q4))/2 - qppr2*cos(q2 + q3 + q4) - qppr1*cos(q2 + q3 + q4) + (qp2*qpr1*sin(q2 + q3 + q4))/2 + qp2*qpr2*sin(q2 + q3 + q4);



            Yr( 4,19) =qppr3 + qppr4;



            Yr( 4,20) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 4,21) =qppr2*sin(2*q5) + qppr3*sin(2*q5) + qppr4*sin(2*q5) + (qppr1*sin(q2 + q3 + q4 - 2*q5))/2 + (qppr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr5*cos(q2 + q3 + q4 - 2*q5))/2 + (qp1*qpr5*cos(q2 + q3 + q4 + 2*q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + qp2*qpr5*cos(2*q5) + qp5*qpr2*cos(2*q5) + qp3*qpr5*cos(2*q5) + qp5*qpr3*cos(2*q5) + qp4*qpr5*cos(2*q5) + qp5*qpr4*cos(2*q5);



            Yr( 4,22) =qppr5*sin(q5) - (qppr1*sin(q2 + q3 + q4 + q5))/2 + (qppr1*sin(q2 + q3 + q4 - q5))/2 + qp5*qpr5*cos(q5) - (qp1*qpr5*cos(q2 + q3 + q4 - q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*cos(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*cos(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 4,23) =qppr2/2 + qppr3/2 + qppr4/2 + (qppr2*cos(2*q5))/2 + (qppr3*cos(2*q5))/2 + (qppr4*cos(2*q5))/2 - (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr5*sin(2*q5))/2 - (qp5*qpr2*sin(2*q5))/2 - (qp3*qpr5*sin(2*q5))/2 - (qp5*qpr3*sin(2*q5))/2 - (qp4*qpr5*sin(2*q5))/2 - (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 4,24) =qppr5*cos(q5) - (qppr1*cos(q2 + q3 + q4 + q5))/2 - (qppr1*cos(q2 + q3 + q4 - q5))/2 - qp5*qpr5*sin(q5) - (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 4,25) =(qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 - (qp1*qpr5*sin(q2 + q3 + q4))/2;



            Yr( 4,26) =qppr2/4 + qppr3/4 + qppr4/4 - (qppr2*cos(2*q5))/4 + (qppr2*cos(2*q6))/4 - (qppr3*cos(2*q5))/4 + (qppr3*cos(2*q6))/4 - (qppr4*cos(2*q5))/4 + (qppr4*cos(2*q6))/4 - (qppr2*cos(2*q5)*cos(2*q6))/4 - (qppr3*cos(2*q5)*cos(2*q6))/4 - (qppr4*cos(2*q5)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q5))/4 + (qp5*qpr2*sin(2*q5))/4 + (qp3*qpr5*sin(2*q5))/4 + (qp5*qpr3*sin(2*q5))/4 - (qp2*qpr6*sin(2*q6))/4 + (qp4*qpr5*sin(2*q5))/4 + (qp5*qpr4*sin(2*q5))/4 - (qp6*qpr2*sin(2*q6))/4 - (qp3*qpr6*sin(2*q6))/4 - (qp6*qpr3*sin(2*q6))/4 - (qp4*qpr6*sin(2*q6))/4 - (qp6*qpr4*sin(2*q6))/4 - (qppr5*sin(2*q6)*sin(q5))/2 - (qp5*qpr5*sin(2*q6)*cos(q5))/2 - (qp5*qpr6*cos(2*q6)*sin(q5))/2 - (qp6*qpr5*cos(2*q6)*sin(q5))/2 + (qp2*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr2*cos(2*q6)*sin(2*q5))/4 + (qp2*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr3*cos(2*q6)*sin(2*q5))/4 + (qp6*qpr2*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr5*cos(2*q6)*sin(2*q5))/4 + (qp5*qpr4*cos(2*q6)*sin(2*q5))/4 + (qp6*qpr3*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr6*cos(2*q5)*sin(2*q6))/4 + (qp6*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2))/8 + (qp1*qpr5*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2))/8 - (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4))/8 - (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3))/8 - (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (3*qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6)*cos(q5))/2 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 - (qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2;



            Yr( 4,27) =(qppr2*cos(2*q5)*sin(2*q6))/2 - (qppr3*sin(2*q6))/2 - (qppr4*sin(2*q6))/2 - (qppr2*sin(2*q6))/2 + (qppr3*cos(2*q5)*sin(2*q6))/2 + (qppr4*cos(2*q5)*sin(2*q6))/2 - (qp2*qpr6*cos(2*q6))/2 - (qp6*qpr2*cos(2*q6))/2 - (qp3*qpr6*cos(2*q6))/2 - (qp6*qpr3*cos(2*q6))/2 - (qp4*qpr6*cos(2*q6))/2 - (qp6*qpr4*cos(2*q6))/2 - qppr5*cos(2*q6)*sin(q5) - qp5*qpr5*cos(2*q6)*cos(q5) + qp5*qpr6*sin(2*q6)*sin(q5) + qp6*qpr5*sin(2*q6)*sin(q5) + (qp2*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr2*cos(2*q5)*cos(2*q6))/2 + (qp3*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr3*cos(2*q5)*cos(2*q6))/2 + (qp4*qpr6*cos(2*q5)*cos(2*q6))/2 + (qp6*qpr4*cos(2*q5)*cos(2*q6))/2 - (qp2*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr2*sin(2*q5)*sin(2*q6))/2 - (qp3*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr3*sin(2*q5)*sin(2*q6))/2 - (qp4*qpr5*sin(2*q5)*sin(2*q6))/2 - (qp5*qpr4*sin(2*q5)*sin(2*q6))/2 + (qppr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6))/4 + (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6))/4 + (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6))/4 + (qp1*qpr5*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (3*qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qppr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qppr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - (qppr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qppr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qppr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + qp1*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4)*cos(q5) + qp1*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4)*cos(q5) + qp1*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3)*cos(q5) + qp1*qpr6*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp1*qpr6*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp1*qpr6*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) + qp6*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr5*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 - (qp5*qpr1*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp1*qpr6*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 - (qp6*qpr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4)*sin(2*q6))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3)*sin(2*q6))/4 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q6))/4 + (qp1*qpr5*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr6*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/4 + qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6)*cos(q5) - qp1*qpr5*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - qp5*qpr1*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - qp1*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp6*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5);



            Yr( 4,28) =qppr5*cos(q5)*sin(q6) - qppr6*cos(q6)*sin(q5) - qppr2*sin(2*q5)*cos(q6) - qppr3*sin(2*q5)*cos(q6) - qppr4*sin(2*q5)*cos(q6) - qp5*qpr5*sin(q5)*sin(q6) + qp6*qpr6*sin(q5)*sin(q6) - qp2*qpr5*cos(2*q5)*cos(q6) - qp5*qpr2*cos(2*q5)*cos(q6) - qp3*qpr5*cos(2*q5)*cos(q6) - qp5*qpr3*cos(2*q5)*cos(q6) - qp4*qpr5*cos(2*q5)*cos(q6) - qp5*qpr4*cos(2*q5)*cos(q6) + (qp2*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr2*sin(2*q5)*sin(q6))/2 + (qp3*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr3*sin(2*q5)*sin(q6))/2 + (qp4*qpr6*sin(2*q5)*sin(q6))/2 + (qp6*qpr4*sin(2*q5)*sin(q6))/2 - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + qppr1*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) + qppr1*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) + qppr1*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - qppr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qppr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qppr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qppr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp1*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp6*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp6*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp1*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*cos(q6))/2 - qp1*qpr5*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp5*qpr1*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr6*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp6*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/2 + qp1*qpr5*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) + qp5*qpr1*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - qp5*qpr1*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(q5)*sin(q6) + qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(q5)*sin(q6) + qp1*qpr5*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp1*qpr5*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp1*qpr5*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + qp5*qpr1*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + qp5*qpr1*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + qp5*qpr1*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp1*qpr6*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp6*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2;



            Yr( 4,29) =qppr2/4 + qppr3/4 + qppr4/4 - (qppr2*cos(2*q5))/4 - (qppr2*cos(2*q6))/4 - (qppr3*cos(2*q5))/4 - (qppr3*cos(2*q6))/4 - (qppr4*cos(2*q5))/4 - (qppr4*cos(2*q6))/4 + (qppr2*cos(2*q5)*cos(2*q6))/4 + (qppr3*cos(2*q5)*cos(2*q6))/4 + (qppr4*cos(2*q5)*cos(2*q6))/4 + (qp2*qpr5*sin(2*q5))/4 + (qp5*qpr2*sin(2*q5))/4 + (qp3*qpr5*sin(2*q5))/4 + (qp5*qpr3*sin(2*q5))/4 + (qp2*qpr6*sin(2*q6))/4 + (qp4*qpr5*sin(2*q5))/4 + (qp5*qpr4*sin(2*q5))/4 + (qp6*qpr2*sin(2*q6))/4 + (qp3*qpr6*sin(2*q6))/4 + (qp6*qpr3*sin(2*q6))/4 + (qp4*qpr6*sin(2*q6))/4 + (qp6*qpr4*sin(2*q6))/4 + (qppr5*sin(2*q6)*sin(q5))/2 + (qp5*qpr5*sin(2*q6)*cos(q5))/2 + (qp5*qpr6*cos(2*q6)*sin(q5))/2 + (qp6*qpr5*cos(2*q6)*sin(q5))/2 - (qp2*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr2*cos(2*q6)*sin(2*q5))/4 - (qp2*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr3*cos(2*q6)*sin(2*q5))/4 - (qp6*qpr2*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr5*cos(2*q6)*sin(2*q5))/4 - (qp5*qpr4*cos(2*q6)*sin(2*q5))/4 - (qp6*qpr3*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr6*cos(2*q5)*sin(2*q6))/4 - (qp6*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 + (qppr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qppr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qppr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qppr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2))/8 + (qp1*qpr5*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr5*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 + (qp5*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*sin(2*q4))/8 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*sin(2*q3))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q2))/8 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4))/8 + (3*qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3))/8 + (3*qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2))/8 - (qp1*qpr5*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr5*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr1*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (3*qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/4 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/4 - (qppr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/4 + (qppr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6)*cos(q5))/2 - (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6)*cos(q5))/2 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr5*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp5*qpr1*cos(2*q5)*cos(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q4))/8 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q3))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2))/8 + (qp1*qpr5*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp5*qpr1*cos(2*q5)*cos(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr6*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp6*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4))/8 - (qp1*qpr6*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp6*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2 - (qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6)*cos(q5))/2 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2 + (qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2;



            Yr( 4,30) =qppr5*cos(q5)*cos(q6) + qppr6*sin(q5)*sin(q6) + qppr2*sin(2*q5)*sin(q6) + qppr3*sin(2*q5)*sin(q6) + qppr4*sin(2*q5)*sin(q6) + qp2*qpr5*cos(2*q5)*sin(q6) + qp5*qpr2*cos(2*q5)*sin(q6) + (qp2*qpr6*sin(2*q5)*cos(q6))/2 + qp3*qpr5*cos(2*q5)*sin(q6) + qp5*qpr3*cos(2*q5)*sin(q6) + (qp6*qpr2*sin(2*q5)*cos(q6))/2 + (qp3*qpr6*sin(2*q5)*cos(q6))/2 + qp4*qpr5*cos(2*q5)*sin(q6) + qp5*qpr4*cos(2*q5)*sin(q6) + (qp6*qpr3*sin(2*q5)*cos(q6))/2 + (qp4*qpr6*sin(2*q5)*cos(q6))/2 + (qp6*qpr4*sin(2*q5)*cos(q6))/2 - qp5*qpr5*cos(q6)*sin(q5) + qp6*qpr6*cos(q6)*sin(q5) + qppr1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + qppr1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + qppr1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + qppr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qppr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qppr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qppr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - (qp1*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp1*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp1*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp6*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp6*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp6*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp1*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp6*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q5)*sin(q6))/2 + qp1*qpr5*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + qp5*qpr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/2 + qp1*qpr5*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) + qp5*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(q6)*sin(q5) - qp1*qpr5*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - qp1*qpr5*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) - qp1*qpr5*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) - qp5*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) - qp5*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) - qp5*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5) + qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*cos(q6)*sin(q5) + qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*cos(q6)*sin(q5) + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp1*qpr6*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp1*qpr6*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp6*qpr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp6*qpr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - qp1*qpr5*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp1*qpr5*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp1*qpr5*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) - qp5*qpr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - (qp1*qpr6*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 4,31) =qppr2/2 + qppr3/2 + qppr4/2 + (qppr2*cos(2*q5))/2 + (qppr3*cos(2*q5))/2 + (qppr4*cos(2*q5))/2 + qppr6*cos(q5) - (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 + (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 - (qp5*qpr6*sin(q5))/2 - (qp6*qpr5*sin(q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr6*sin(q2 + q3 + q4 - q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp6*qpr1*sin(q2 + q3 + q4 - q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp2*qpr5*sin(2*q5))/2 - (qp5*qpr2*sin(2*q5))/2 - (qp3*qpr5*sin(2*q5))/2 - (qp5*qpr3*sin(2*q5))/2 - (qp4*qpr5*sin(2*q5))/2 - (qp5*qpr4*sin(2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4 + (qp1*qpr6*sin(q2 + q3 + q4 + q5))/4 + (qp6*qpr1*sin(q2 + q3 + q4 + q5))/4;



            Yr( 4,32) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(q2 + q3 + q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 4,33) =0;



            Yr( 4,34) =0;



            Yr( 4,35) =0;



            Yr( 4,36) =0;



            Yr( 4,37) =0;



            Yr( 4,38) =0;



            Yr( 4,39) =0;



            Yr( 4,40) =0;



            Yr( 4,41) =0;



            Yr( 4,42) =0;



            Yr( 4,43) =0;



            Yr( 4,44) =qppr2/2 + qppr3/2 + qppr4/2 - (qppr2*cos(2*q5))/2 - (qppr3*cos(2*q5))/2 - (qppr4*cos(2*q5))/2 + (qppr1*cos(q2 + q3 + q4 - 2*q5))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr5*sin(q2 + q3 + q4))/2 - (qp5*qpr1*sin(q2 + q3 + q4))/2 + (qp2*qpr5*sin(2*q5))/2 + (qp5*qpr2*sin(2*q5))/2 + (qp3*qpr5*sin(2*q5))/2 + (qp5*qpr3*sin(2*q5))/2 + (qp4*qpr5*sin(2*q5))/2 + (qp5*qpr4*sin(2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/4;



            Yr( 4,45) =0;



            Yr( 4,46) =0;



            Yr( 4,47) =0;



            Yr( 4,48) =0;



            Yr( 4,49) =qppr2 + qppr3 + qppr4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 4,50) =0;



            Yr( 4,51) =0;



            Yr( 4,52) =0;



            Yr( 4,53) =0;



            Yr( 4,54) =0;



            Yr( 4,55) =-sin(q2 + q3 + q4)*cos(q1)*sin(q5);



            Yr( 4,56) =-sin(q2 + q3 + q4)*sin(q1)*sin(q5);



            Yr( 4,57) =sin(q2 + q3 + q4 - q5)/2 - sin(q2 + q3 + q4 + q5)/2;



            Yr( 4,58) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 4,59) =sin(q5)*(qppr2*cos(q3 + q4) + (qp1*qpr1*sin(2*q2 + q3 + q4))/2 + (qp1*qpr1*sin(q3 + q4))/2 + qp2*qpr2*sin(q3 + q4));



            Yr( 4,60) =qppr2 + qppr3 + qppr4 - qppr2*cos(2*q5) - qppr3*cos(2*q5) - qppr4*cos(2*q5) + (qppr1*cos(q2 + q3 + q4 - 2*q5))/2 - (qppr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 - 2*q5))/2 + (qp1*qpr5*sin(q2 + q3 + q4 + 2*q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - 2*q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 - qp1*qpr5*sin(q2 + q3 + q4) - qp5*qpr1*sin(q2 + q3 + q4) + qp2*qpr5*sin(2*q5) + qp5*qpr2*sin(2*q5) + qp3*qpr5*sin(2*q5) + qp5*qpr3*sin(2*q5) + qp4*qpr5*sin(2*q5) + qp5*qpr4*sin(2*q5) + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4))/2;



            Yr( 4,61) =(sin(q5)*(2*qppr2*cos(q4) + 2*qppr3*cos(q4) + qp1*qpr1*sin(q4) + 2*qp2*qpr2*sin(q4) + 2*qp2*qpr3*sin(q4) + 2*qp3*qpr2*sin(q4) + 2*qp3*qpr3*sin(q4) + qp1*qpr1*sin(2*q2 + 2*q3 + q4)))/2;



            Yr( 4,62) =(qppr1*cos(q2 + q3 + q4 + q5))/2 - qppr5*cos(q5) + (qppr1*cos(q2 + q3 + q4 - q5))/2 + qp5*qpr5*sin(q5) + (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 4,63) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 4,64) =(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 4,65) =0;



            Yr( 4,66) =0;



            Yr( 4,67) =0;



            Yr( 4,68) =0;



            Yr( 4,69) =0;



            Yr( 4,70) =0;



            Yr( 4,71) =0;



            Yr( 4,72) =0;



            Yr( 4,73) =0;



            Yr( 4,74) =0;



            Yr( 4,75) =0;



            Yr( 4,76) =-sin(q2 + q3 + q4)*cos(q1)*sin(q5);



            Yr( 4,77) =0;



            Yr( 4,78) =0;



            Yr( 4,79) =0;



            Yr( 4,80) =0;



            Yr( 4,81) =- cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2;



            Yr( 4,82) =0;



            Yr( 4,83) =0;



            Yr( 4,84) =0;



            Yr( 4,85) =0;



            Yr( 4,86) =0;



            Yr( 4,87) =0;



            Yr( 4,88) =0;



            Yr( 4,89) =0;



            Yr( 4,90) =0;



            Yr( 4,91) =0;



            Yr( 4,92) =0;



            Yr( 4,93) =0;



            Yr( 4,94) =0;



            Yr( 4,95) =0;



            Yr( 4,96) =0;



            Yr( 4,97) =0;



            Yr( 4,98) =-sin(q2 + q3 + q4)*sin(q1)*sin(q5);



            Yr( 4,99) =0;



            Yr( 4,100) =0;



            Yr( 4,101) =0;



            Yr( 4,102) =0;



            Yr( 4,103) =sin(q2 - q1 + q3 + q4)/2 - sin(q1 + q2 + q3 + q4)/2;



            Yr( 4,104) =0;



            Yr( 4,105) =0;



            Yr( 4,106) =0;



            Yr( 4,107) =0;



            Yr( 4,108) =0;



            Yr( 4,109) =0;



            Yr( 4,110) =0;



            Yr( 4,111) =0;



            Yr( 4,112) =0;



            Yr( 4,113) =0;



            Yr( 4,114) =sin(q2 + q3 + q4 - q5)/2 - sin(q2 + q3 + q4 + q5)/2;



            Yr( 4,115) =0;



            Yr( 4,116) =0;



            Yr( 4,117) =0;



            Yr( 4,118) =0;



            Yr( 4,119) =sin(q2 + q3 + q4);



            Yr( 4,120) =0;



            Yr( 4,121) =0;



            Yr( 4,122) =0;



            Yr( 4,123) =0;



            Yr( 4,124) =0;



            Yr( 4,125) =0;



            Yr( 4,126) =0;



            Yr( 4,127) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 4,128) =0;



            Yr( 4,129) =0;



            Yr( 4,130) =0;



            Yr( 4,131) =sin(q5)*(qppr2*cos(q3 + q4) + (qp1*qpr1*sin(2*q2 + q3 + q4))/2 + (qp1*qpr1*sin(q3 + q4))/2 + qp2*qpr2*sin(q3 + q4));



            Yr( 4,132) =0;



            Yr( 4,133) =0;



            Yr( 4,134) =0;



            Yr( 4,135) =(sin(q5)*(2*qppr2*cos(q4) + 2*qppr3*cos(q4) + qp1*qpr1*sin(q4) + 2*qp2*qpr2*sin(q4) + 2*qp2*qpr3*sin(q4) + 2*qp3*qpr2*sin(q4) + 2*qp3*qpr3*sin(q4) + qp1*qpr1*sin(2*q2 + 2*q3 + q4)))/2;



            Yr( 4,136) =0;



            Yr( 4,137) =qppr1*cos(q2 + q3 + q4);



            Yr( 4,138) =0;



            Yr( 4,139) =0;



            Yr( 4,140) =0;



            Yr( 4,141) =0;



            Yr( 4,142) =(qp1*qpr1*cos(2*q2 + q3 + q4))/2 - qppr2*sin(q3 + q4) + (qp1*qpr1*cos(q3 + q4))/2 + qp2*qpr2*cos(q3 + q4);



            Yr( 4,143) =0;



            Yr( 4,144) =0;



            Yr( 4,145) =0;



            Yr( 4,146) =0;



            Yr( 4,147) =(qppr1*cos(q2 + q3 + q4 + q5))/2 - qppr5*cos(q5) + (qppr1*cos(q2 + q3 + q4 - q5))/2 + qp5*qpr5*sin(q5) + (qp1*qpr5*sin(q2 + q3 + q4 - q5))/2 + (qp5*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/2 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/2 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/2;



            Yr( 4,148) =0;



            Yr( 4,149) =0;



            Yr( 4,150) =0;



            Yr( 4,151) =(qp1*qpr1*cos(q4))/2 - qppr3*sin(q4) - qppr2*sin(q4) + qp2*qpr2*cos(q4) + qp2*qpr3*cos(q4) + qp3*qpr2*cos(q4) + qp3*qpr3*cos(q4) + (qp1*qpr1*cos(2*q2 + 2*q3 + q4))/2;



            Yr( 4,152) =0;



            Yr( 4,153) =-(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 4,154) =0;



            Yr( 4,155) =0;



            Yr( 4,156) =(qppr1*(cos(q2 + q3 + q4 + q5) - cos(q2 + q3 + q4 - q5)))/2;



            Yr( 4,157) =0;



            Yr( 4,158) =0;



            Yr( 4,159) =0;



            Yr( 4,160) =qppr1*cos(q2 + q3 + q4);



            Yr( 4,161) =-qppr1*cos(q2 + q3 + q4);



            Yr( 4,162) =0;



            Yr( 4,163) =0;



            Yr( 5,1) =0;



            Yr( 5,2) =0;



            Yr( 5,3) =0;



            Yr( 5,4) =0;



            Yr( 5,5) =0;



            Yr( 5,6) =0;



            Yr( 5,7) =0;



            Yr( 5,8) =0;



            Yr( 5,9) =0;



            Yr( 5,10) =0;



            Yr( 5,11) =0;



            Yr( 5,12) =0;



            Yr( 5,13) =0;



            Yr( 5,14) =0;



            Yr( 5,15) =0;



            Yr( 5,16) =0;



            Yr( 5,17) =0;



            Yr( 5,18) =0;



            Yr( 5,19) =0;



            Yr( 5,20) =(qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr1*sin(2*q5))/4 - (qp2*qpr2*sin(2*q5))/2 - (qp2*qpr3*sin(2*q5))/2 - (qp3*qpr2*sin(2*q5))/2 - (qp2*qpr4*sin(2*q5))/2 - (qp3*qpr3*sin(2*q5))/2 - (qp4*qpr2*sin(2*q5))/2 - (qp3*qpr4*sin(2*q5))/2 - (qp4*qpr3*sin(2*q5))/2 - (qp4*qpr4*sin(2*q5))/2;



            Yr( 5,21) =(qp1*qpr2*cos(q2 + q3 + q4 - 2*q5))/2 - (qp1*qpr2*cos(q2 + q3 + q4 + 2*q5))/2 + (qp2*qpr1*cos(q2 + q3 + q4 - 2*q5))/2 - (qp2*qpr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr3*cos(q2 + q3 + q4 - 2*q5))/2 - (qp1*qpr3*cos(q2 + q3 + q4 + 2*q5))/2 + (qp3*qpr1*cos(q2 + q3 + q4 - 2*q5))/2 - (qp3*qpr1*cos(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr4*cos(q2 + q3 + q4 - 2*q5))/2 - (qp1*qpr4*cos(q2 + q3 + q4 + 2*q5))/2 + (qp4*qpr1*cos(q2 + q3 + q4 - 2*q5))/2 - (qp4*qpr1*cos(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + (qp1*qpr1*cos(2*q5))/2 - qp2*qpr2*cos(2*q5) - qp2*qpr3*cos(2*q5) - qp3*qpr2*cos(2*q5) - qp2*qpr4*cos(2*q5) - qp3*qpr3*cos(2*q5) - qp4*qpr2*cos(2*q5) - qp3*qpr4*cos(2*q5) - qp4*qpr3*cos(2*q5) - qp4*qpr4*cos(2*q5);



            Yr( 5,22) =(qppr1*sin(q2 + q3 + q4 + q5))/2 + qppr2*sin(q5) + qppr3*sin(q5) + qppr4*sin(q5) + (qppr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr2*cos(q2 + q3 + q4 - q5))/2 + (qp2*qpr1*cos(q2 + q3 + q4 - q5))/2 + (qp1*qpr3*cos(q2 + q3 + q4 - q5))/2 + (qp3*qpr1*cos(q2 + q3 + q4 - q5))/2 + (qp1*qpr4*cos(q2 + q3 + q4 - q5))/2 + (qp4*qpr1*cos(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 - q5))/4 + (qp1*qpr2*cos(q2 + q3 + q4 + q5))/2 + (qp2*qpr1*cos(q2 + q3 + q4 + q5))/2 + (qp1*qpr3*cos(q2 + q3 + q4 + q5))/2 + (qp3*qpr1*cos(q2 + q3 + q4 + q5))/2 + (qp1*qpr4*cos(q2 + q3 + q4 + q5))/2 + (qp4*qpr1*cos(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*cos(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 5,23) =(qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr1*sin(2*q5))/4 + (qp2*qpr2*sin(2*q5))/2 + (qp2*qpr3*sin(2*q5))/2 + (qp3*qpr2*sin(2*q5))/2 + (qp2*qpr4*sin(2*q5))/2 + (qp3*qpr3*sin(2*q5))/2 + (qp4*qpr2*sin(2*q5))/2 + (qp3*qpr4*sin(2*q5))/2 + (qp4*qpr3*sin(2*q5))/2 + (qp4*qpr4*sin(2*q5))/2;



            Yr( 5,24) =(qppr1*cos(q2 + q3 + q4 + q5))/2 + qppr2*cos(q5) + qppr3*cos(q5) + qppr4*cos(q5) - (qppr1*cos(q2 + q3 + q4 - q5))/2 + (qp1*qpr2*sin(q2 + q3 + q4 - q5))/2 + (qp2*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr3*sin(q2 + q3 + q4 - q5))/2 + (qp3*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr4*sin(q2 + q3 + q4 - q5))/2 + (qp4*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 - (qp1*qpr2*sin(q2 + q3 + q4 + q5))/2 - (qp2*qpr1*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr3*sin(q2 + q3 + q4 + q5))/2 - (qp3*qpr1*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr4*sin(q2 + q3 + q4 + q5))/2 - (qp4*qpr1*sin(q2 + q3 + q4 + q5))/2 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 5,25) =qppr5 - qppr1*cos(q2 + q3 + q4) + (qp1*qpr2*sin(q2 + q3 + q4))/2 + (qp2*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr3*sin(q2 + q3 + q4))/2 + (qp3*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr4*sin(q2 + q3 + q4))/2 + (qp4*qpr1*sin(q2 + q3 + q4))/2;



            Yr( 5,26) =qppr5 - qppr5*pow(cos(q6),2) + (qp2*qpr6*sin(q5))/2 + (qp6*qpr2*sin(q5))/2 + (qp3*qpr6*sin(q5))/2 + (qp6*qpr3*sin(q5))/2 + (qp4*qpr6*sin(q5))/2 + (qp6*qpr4*sin(q5))/2 + (qp5*qpr6*sin(2*q6))/2 + (qp6*qpr5*sin(2*q6))/2 - qppr1*cos(q2)*cos(q3)*cos(q4) - qp2*qpr6*pow(cos(q6),2)*sin(q5) - qp6*qpr2*pow(cos(q6),2)*sin(q5) - qp3*qpr6*pow(cos(q6),2)*sin(q5) - qp6*qpr3*pow(cos(q6),2)*sin(q5) - qp4*qpr6*pow(cos(q6),2)*sin(q5) - qp6*qpr4*pow(cos(q6),2)*sin(q5) + qppr1*cos(q2)*sin(q3)*sin(q4) + qppr1*cos(q3)*sin(q2)*sin(q4) + qppr1*cos(q4)*sin(q2)*sin(q3) - qppr2*cos(q6)*sin(q5)*sin(q6) - qppr3*cos(q6)*sin(q5)*sin(q6) - qppr4*cos(q6)*sin(q5)*sin(q6) + qp1*qpr1*cos(q5)*pow(cos(q6),2)*sin(q5) - qp2*qpr2*cos(q5)*pow(cos(q6),2)*sin(q5) - qp2*qpr3*cos(q5)*pow(cos(q6),2)*sin(q5) - qp3*qpr2*cos(q5)*pow(cos(q6),2)*sin(q5) - qp2*qpr4*cos(q5)*pow(cos(q6),2)*sin(q5) - qp3*qpr3*cos(q5)*pow(cos(q6),2)*sin(q5) - qp4*qpr2*cos(q5)*pow(cos(q6),2)*sin(q5) - qp3*qpr4*cos(q5)*pow(cos(q6),2)*sin(q5) - qp4*qpr3*cos(q5)*pow(cos(q6),2)*sin(q5) - qp4*qpr4*cos(q5)*pow(cos(q6),2)*sin(q5) + qppr1*cos(q2)*cos(q3)*cos(q4)*pow(cos(q6),2) - qppr1*cos(q2)*pow(cos(q6),2)*sin(q3)*sin(q4) - qppr1*cos(q3)*pow(cos(q6),2)*sin(q2)*sin(q4) - qppr1*cos(q4)*pow(cos(q6),2)*sin(q2)*sin(q3) + (qp1*qpr2*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr2*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr2*cos(q3)*cos(q4)*sin(q2))/2 + (qp2*qpr1*cos(q2)*cos(q3)*sin(q4))/2 + (qp2*qpr1*cos(q2)*cos(q4)*sin(q3))/2 + (qp2*qpr1*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr3*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr3*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr3*cos(q3)*cos(q4)*sin(q2))/2 + (qp3*qpr1*cos(q2)*cos(q3)*sin(q4))/2 + (qp3*qpr1*cos(q2)*cos(q4)*sin(q3))/2 + (qp3*qpr1*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr4*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr4*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr4*cos(q3)*cos(q4)*sin(q2))/2 + (qp4*qpr1*cos(q2)*cos(q3)*sin(q4))/2 + (qp4*qpr1*cos(q2)*cos(q4)*sin(q3))/2 + (qp4*qpr1*cos(q3)*cos(q4)*sin(q2))/2 - (qp1*qpr2*sin(q2)*sin(q3)*sin(q4))/2 - (qp2*qpr1*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr3*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr1*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr4*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr1*sin(q2)*sin(q3)*sin(q4))/2 - qp1*qpr1*pow(cos(q2),2)*cos(q5)*pow(cos(q6),2)*sin(q5) - qp1*qpr1*pow(cos(q3),2)*cos(q5)*pow(cos(q6),2)*sin(q5) - qp1*qpr1*pow(cos(q4),2)*cos(q5)*pow(cos(q6),2)*sin(q5) + (qp1*qpr6*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2 + (qp1*qpr6*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2 + (qp1*qpr6*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2 + (qp6*qpr1*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2 + (qp6*qpr1*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2 + (qp6*qpr1*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2 - (qp1*qpr6*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp6*qpr1*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2 + qppr1*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp1*qpr2*cos(q2)*cos(q3)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q4) - qp1*qpr2*cos(q2)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q3) - qp1*qpr2*cos(q3)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2) - qp2*qpr1*cos(q2)*cos(q3)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q4) - qp2*qpr1*cos(q2)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q3) - qp2*qpr1*cos(q3)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2) - qp1*qpr3*cos(q2)*cos(q3)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q4) - qp1*qpr3*cos(q2)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q3) - qp1*qpr3*cos(q3)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2) - qp3*qpr1*cos(q2)*cos(q3)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q4) - qp3*qpr1*cos(q2)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q3) - qp3*qpr1*cos(q3)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2) - qp1*qpr4*cos(q2)*cos(q3)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q4) - qp1*qpr4*cos(q2)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q3) - qp1*qpr4*cos(q3)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2) - qp4*qpr1*cos(q2)*cos(q3)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q4) - qp4*qpr1*cos(q2)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q3) - qp4*qpr1*cos(q3)*cos(q4)*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2) + qp1*qpr2*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) + qp2*qpr1*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) + qp1*qpr3*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) + qp3*qpr1*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) + qp1*qpr4*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) + qp4*qpr1*pow(cos(q5),2)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) - qp1*qpr6*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q6) - qp6*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q6) + 2*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*cos(q5)*pow(cos(q6),2)*sin(q5) + 2*qp1*qpr1*pow(cos(q2),2)*pow(cos(q4),2)*cos(q5)*pow(cos(q6),2)*sin(q5) + 2*qp1*qpr1*pow(cos(q3),2)*pow(cos(q4),2)*cos(q5)*pow(cos(q6),2)*sin(q5) + qp1*qpr1*cos(q2)*cos(q6)*sin(q2)*sin(q5)*sin(q6) + qp1*qpr1*cos(q3)*cos(q6)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr1*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr6*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp1*qpr6*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp1*qpr6*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp6*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp6*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp6*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q6) - qp1*qpr6*cos(q2)*cos(q3)*cos(q5)*pow(cos(q6),2)*sin(q4) - qp1*qpr6*cos(q2)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q3) - qp1*qpr6*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q2) - qp6*qpr1*cos(q2)*cos(q3)*cos(q5)*pow(cos(q6),2)*sin(q4) - qp6*qpr1*cos(q2)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q3) - qp6*qpr1*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q2) - qppr1*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*sin(q6) - qppr1*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q6) - qppr1*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q6) + qp1*qpr6*cos(q5)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) + qp6*qpr1*cos(q5)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q4) - qp1*qpr2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6) - qp2*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6) - qp1*qpr3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6) - qp3*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6) - qp1*qpr4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6) - qp4*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6) + qp1*qpr2*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp1*qpr2*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp1*qpr2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp2*qpr1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp2*qpr1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp2*qpr1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp1*qpr3*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp1*qpr3*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp1*qpr3*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp3*qpr1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp3*qpr1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp3*qpr1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp1*qpr4*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp1*qpr4*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp1*qpr4*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) + qp4*qpr1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6) + qp4*qpr1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6) + qp4*qpr1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6) - 4*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*pow(cos(q4),2)*cos(q5)*pow(cos(q6),2)*sin(q5) - 2*qp1*qpr1*cos(q2)*pow(cos(q3),2)*cos(q6)*sin(q2)*sin(q5)*sin(q6) - 2*qp1*qpr1*cos(q2)*pow(cos(q4),2)*cos(q6)*sin(q2)*sin(q5)*sin(q6) - 2*qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q6)*sin(q3)*sin(q5)*sin(q6) - 2*qp1*qpr1*cos(q3)*pow(cos(q4),2)*cos(q6)*sin(q3)*sin(q5)*sin(q6) - 2*qp1*qpr1*pow(cos(q2),2)*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6) - 2*qp1*qpr1*pow(cos(q3),2)*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6) + 4*qp1*qpr1*cos(q2)*pow(cos(q3),2)*pow(cos(q4),2)*cos(q6)*sin(q2)*sin(q5)*sin(q6) + 4*qp1*qpr1*pow(cos(q2),2)*cos(q3)*pow(cos(q4),2)*cos(q6)*sin(q3)*sin(q5)*sin(q6) + 4*qp1*qpr1*pow(cos(q2),2)*pow(cos(q3),2)*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6) - 2*qp1*qpr1*cos(q2)*cos(q3)*cos(q5)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q5) - 2*qp1*qpr1*cos(q2)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q2)*sin(q4)*sin(q5) - 2*qp1*qpr1*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q3)*sin(q4)*sin(q5) + 4*qp1*qpr1*cos(q2)*cos(q3)*pow(cos(q4),2)*cos(q5)*pow(cos(q6),2)*sin(q2)*sin(q3)*sin(q5) + 4*qp1*qpr1*cos(q2)*pow(cos(q3),2)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q2)*sin(q4)*sin(q5) + 4*qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2)*sin(q3)*sin(q4)*sin(q5) - 4*qp1*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);



            Yr( 5,27) =qppr5*sin(2*q6) + qp5*qpr6*cos(2*q6) + qp6*qpr5*cos(2*q6) - qppr2*cos(2*q6)*sin(q5) - qppr3*cos(2*q6)*sin(q5) - qppr4*cos(2*q6)*sin(q5) + qp2*qpr6*sin(2*q6)*sin(q5) + qp6*qpr2*sin(2*q6)*sin(q5) + qp3*qpr6*sin(2*q6)*sin(q5) + qp6*qpr3*sin(2*q6)*sin(q5) + qp4*qpr6*sin(2*q6)*sin(q5) + qp6*qpr4*sin(2*q6)*sin(q5) - (qp1*qpr1*sin(2*q5)*sin(2*q6))/4 + (qp2*qpr2*sin(2*q5)*sin(2*q6))/2 + (qp2*qpr3*sin(2*q5)*sin(2*q6))/2 + (qp3*qpr2*sin(2*q5)*sin(2*q6))/2 + (qp2*qpr4*sin(2*q5)*sin(2*q6))/2 + (qp3*qpr3*sin(2*q5)*sin(2*q6))/2 + (qp4*qpr2*sin(2*q5)*sin(2*q6))/2 + (qp3*qpr4*sin(2*q5)*sin(2*q6))/2 + (qp4*qpr3*sin(2*q5)*sin(2*q6))/2 + (qp4*qpr4*sin(2*q5)*sin(2*q6))/2 - qppr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4) + qppr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4) + qppr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4) + qppr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3) + (qp1*qpr2*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr2*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr2*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp2*qpr1*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp2*qpr1*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp2*qpr1*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr3*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr3*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr3*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp3*qpr1*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp3*qpr1*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp3*qpr1*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr4*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr4*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr4*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp4*qpr1*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp4*qpr1*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp4*qpr1*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + qp1*qpr6*cos(2*q6)*cos(q2)*sin(q3)*sin(q4) + qp1*qpr6*cos(2*q6)*cos(q3)*sin(q2)*sin(q4) + qp1*qpr6*cos(2*q6)*cos(q4)*sin(q2)*sin(q3) + qp6*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4) + qp6*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4) + qp6*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3) - (qp1*qpr2*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp2*qpr1*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr3*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr1*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr4*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr1*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - qppr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - qppr1*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - qppr1*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + qppr1*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - qp1*qpr6*cos(2*q6)*cos(q2)*cos(q3)*cos(q4) - qp6*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4) + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*sin(q5))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*sin(q5))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(q5))/2 - qp1*qpr6*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - qp6*qpr1*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - (qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5))/2 + (qp1*qpr2*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr2*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr2*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp2*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp2*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp2*qpr1*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr3*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr3*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr3*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp3*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp3*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp3*qpr1*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr4*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr4*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr4*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp4*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/2 + (qp4*qpr1*cos(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/2 + (qp4*qpr1*cos(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(2*q6))/4 - (qp1*qpr2*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp2*qpr1*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr3*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr1*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr4*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr1*cos(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(2*q6))/4 - (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(2*q6))/4 - (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(2*q6))/4 - qp1*qpr2*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp2*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp1*qpr3*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp3*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp1*qpr4*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - qp4*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + qp1*qpr2*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + qp1*qpr2*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp1*qpr2*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + qp2*qpr1*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + qp2*qpr1*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp2*qpr1*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + qp1*qpr3*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + qp1*qpr3*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp1*qpr3*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + qp3*qpr1*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + qp3*qpr1*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp3*qpr1*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + qp1*qpr4*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + qp1*qpr4*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp1*qpr4*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + qp4*qpr1*cos(2*q6)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + qp4*qpr1*cos(2*q6)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp4*qpr1*cos(2*q6)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + qp1*qpr6*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) + qp1*qpr6*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + qp1*qpr6*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + qp6*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) + qp6*qpr1*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + qp6*qpr1*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2);



            Yr( 5,28) =qppr6*sin(q6) + qp6*qpr6*cos(q6) + qppr2*cos(q5)*sin(q6) + qppr3*cos(q5)*sin(q6) + qppr4*cos(q5)*sin(q6) - (qp1*qpr1*cos(2*q5)*cos(q6))/2 + qp2*qpr2*cos(2*q5)*cos(q6) + qp2*qpr3*cos(2*q5)*cos(q6) + qp3*qpr2*cos(2*q5)*cos(q6) + qp2*qpr4*cos(2*q5)*cos(q6) + qp3*qpr3*cos(2*q5)*cos(q6) + qp4*qpr2*cos(2*q5)*cos(q6) + qp3*qpr4*cos(2*q5)*cos(q6) + qp4*qpr3*cos(2*q5)*cos(q6) + qp4*qpr4*cos(2*q5)*cos(q6) + qp2*qpr6*cos(q5)*cos(q6) + qp6*qpr2*cos(q5)*cos(q6) + qp3*qpr6*cos(q5)*cos(q6) + qp6*qpr3*cos(q5)*cos(q6) + qp4*qpr6*cos(q5)*cos(q6) + qp6*qpr4*cos(q5)*cos(q6) - qppr1*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) - qppr1*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) - qppr1*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) + qppr1*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - (qp1*qpr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*cos(q6))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*cos(q6))/2 - (qp1*qpr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*cos(q6))/2 + qp1*qpr2*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qp2*qpr1*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qp1*qpr3*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qp3*qpr1*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qp1*qpr4*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + qp4*qpr1*sin(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - qp1*qpr2*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp2*qpr1*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp1*qpr3*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp3*qpr1*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp1*qpr4*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp4*qpr1*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q6) - qp1*qpr6*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) - qp1*qpr6*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - qp1*qpr6*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) - qp6*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) - qp6*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - qp6*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*sin(q6))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*sin(q6))/2 + qp1*qpr2*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr2*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr2*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp2*qpr1*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp2*qpr1*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) + qp2*qpr1*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr3*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr3*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr3*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp3*qpr1*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp3*qpr1*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) + qp3*qpr1*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr4*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr4*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr4*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp4*qpr1*cos(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp4*qpr1*cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6) + qp4*qpr1*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr6*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + qp6*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(q6))/2 - qp1*qpr2*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qp1*qpr2*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qp1*qpr2*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qp2*qpr1*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qp2*qpr1*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qp2*qpr1*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qp1*qpr3*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qp1*qpr3*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qp1*qpr3*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qp3*qpr1*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qp3*qpr1*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qp3*qpr1*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qp1*qpr4*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qp1*qpr4*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qp1*qpr4*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - qp4*qpr1*sin(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - qp4*qpr1*sin(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3) - qp4*qpr1*sin(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2);



            Yr( 5,29) =qppr5/2 + (qppr5*cos(2*q6))/2 + (qppr1*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (qppr1*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (qppr1*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (qppr1*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (qppr1*cos(q2 + q3 + q4 - 2*q6))/4 - (qppr1*cos(q2 + q3 + q4 + 2*q6))/4 + (qppr2*cos(q5 - 2*q6))/4 - (qppr2*cos(q5 + 2*q6))/4 + (qppr3*cos(q5 - 2*q6))/4 - (qppr3*cos(q5 + 2*q6))/4 + (qppr4*cos(q5 - 2*q6))/4 - (qppr4*cos(q5 + 2*q6))/4 - (qppr1*cos(q2 + q3 + q4))/2 - (qp1*qpr2*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp1*qpr2*sin(q2 + q3 + q4 - q5 + 2*q6))/8 - (qp2*qpr1*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp2*qpr1*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp1*qpr3*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp1*qpr3*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp3*qpr1*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp3*qpr1*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp1*qpr4*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp1*qpr4*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (qp4*qpr1*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp4*qpr1*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (qp1*qpr6*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp1*qpr6*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp6*qpr1*sin(q2 + q3 + q4 - q5 - 2*q6))/8 + (qp6*qpr1*sin(q2 + q3 + q4 - q5 + 2*q6))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 - 2*q6))/16 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5 + 2*q6))/16 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 - 2*q6))/32 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5 + 2*q6))/32 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 - 2*q6))/32 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5 + 2*q6))/32 - (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/8 - (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/8 - (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/8 - (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/8 + (qp1*qpr2*sin(q2 + q3 + q4 - 2*q6))/8 + (qp1*qpr2*sin(q2 + q3 + q4 + 2*q6))/8 - (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/8 - (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/8 + (qp2*qpr1*sin(q2 + q3 + q4 - 2*q6))/8 + (qp2*qpr1*sin(q2 + q3 + q4 + 2*q6))/8 - (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/8 - (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/8 + (qp1*qpr3*sin(q2 + q3 + q4 - 2*q6))/8 + (qp1*qpr3*sin(q2 + q3 + q4 + 2*q6))/8 - (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/8 - (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/8 + (qp3*qpr1*sin(q2 + q3 + q4 - 2*q6))/8 + (qp3*qpr1*sin(q2 + q3 + q4 + 2*q6))/8 - (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/8 - (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/8 + (qp1*qpr4*sin(q2 + q3 + q4 - 2*q6))/8 + (qp1*qpr4*sin(q2 + q3 + q4 + 2*q6))/8 + (qp4*qpr1*sin(q2 + q3 + q4 - 2*q6))/8 + (qp4*qpr1*sin(q2 + q3 + q4 + 2*q6))/8 - (qp1*qpr6*sin(q2 + q3 + q4 - 2*q6))/4 + (qp1*qpr6*sin(q2 + q3 + q4 + 2*q6))/4 - (qp6*qpr1*sin(q2 + q3 + q4 - 2*q6))/4 + (qp6*qpr1*sin(q2 + q3 + q4 + 2*q6))/4 + (qp2*qpr6*sin(q5 - 2*q6))/4 + (qp2*qpr6*sin(q5 + 2*q6))/4 + (qp6*qpr2*sin(q5 - 2*q6))/4 + (qp6*qpr2*sin(q5 + 2*q6))/4 + (qp3*qpr6*sin(q5 - 2*q6))/4 + (qp3*qpr6*sin(q5 + 2*q6))/4 + (qp6*qpr3*sin(q5 - 2*q6))/4 + (qp6*qpr3*sin(q5 + 2*q6))/4 + (qp4*qpr6*sin(q5 - 2*q6))/4 + (qp4*qpr6*sin(q5 + 2*q6))/4 + (qp6*qpr4*sin(q5 - 2*q6))/4 + (qp6*qpr4*sin(q5 + 2*q6))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/16 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/16 + (qp1*qpr2*sin(q2 + q3 + q4))/4 + (qp2*qpr1*sin(q2 + q3 + q4))/4 + (qp1*qpr3*sin(q2 + q3 + q4))/4 + (qp3*qpr1*sin(q2 + q3 + q4))/4 + (qp1*qpr4*sin(q2 + q3 + q4))/4 + (qp4*qpr1*sin(q2 + q3 + q4))/4 + (qp1*qpr1*sin(2*q5))/8 - (qp2*qpr2*sin(2*q5))/4 - (qp2*qpr3*sin(2*q5))/4 - (qp3*qpr2*sin(2*q5))/4 - (qp2*qpr4*sin(2*q5))/4 - (qp3*qpr3*sin(2*q5))/4 - (qp4*qpr2*sin(2*q5))/4 - (qp3*qpr4*sin(2*q5))/4 - (qp4*qpr3*sin(2*q5))/4 - (qp4*qpr4*sin(2*q5))/4 - (qp5*qpr6*sin(2*q6))/2 - (qp6*qpr5*sin(2*q6))/2 - (qp1*qpr2*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp1*qpr2*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp2*qpr1*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp2*qpr1*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp1*qpr3*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp1*qpr3*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp3*qpr1*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp3*qpr1*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp1*qpr4*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp1*qpr4*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp4*qpr1*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp4*qpr1*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp1*qpr6*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp1*qpr6*sin(q2 + q3 + q4 + q5 + 2*q6))/8 + (qp6*qpr1*sin(q2 + q3 + q4 + q5 - 2*q6))/8 + (qp6*qpr1*sin(q2 + q3 + q4 + q5 + 2*q6))/8 - (qp1*qpr1*sin(2*q5 - 2*q6))/16 - (qp1*qpr1*sin(2*q5 + 2*q6))/16 + (qp2*qpr2*sin(2*q5 - 2*q6))/8 + (qp2*qpr2*sin(2*q5 + 2*q6))/8 + (qp2*qpr3*sin(2*q5 - 2*q6))/8 + (qp2*qpr3*sin(2*q5 + 2*q6))/8 + (qp3*qpr2*sin(2*q5 - 2*q6))/8 + (qp3*qpr2*sin(2*q5 + 2*q6))/8 + (qp2*qpr4*sin(2*q5 - 2*q6))/8 + (qp2*qpr4*sin(2*q5 + 2*q6))/8 + (qp3*qpr3*sin(2*q5 - 2*q6))/8 + (qp3*qpr3*sin(2*q5 + 2*q6))/8 + (qp4*qpr2*sin(2*q5 - 2*q6))/8 + (qp4*qpr2*sin(2*q5 + 2*q6))/8 + (qp3*qpr4*sin(2*q5 - 2*q6))/8 + (qp3*qpr4*sin(2*q5 + 2*q6))/8 + (qp4*qpr3*sin(2*q5 - 2*q6))/8 + (qp4*qpr3*sin(2*q5 + 2*q6))/8 + (qp4*qpr4*sin(2*q5 - 2*q6))/8 + (qp4*qpr4*sin(2*q5 + 2*q6))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 - 2*q6))/16 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5 + 2*q6))/16;



            Yr( 5,30) =qppr6*cos(q6) - qp6*qpr6*sin(q6) + qppr2*cos(q5)*cos(q6) + qppr3*cos(q5)*cos(q6) + qppr4*cos(q5)*cos(q6) + (qp1*qpr1*cos(2*q5)*sin(q6))/2 - qp2*qpr2*cos(2*q5)*sin(q6) - qp2*qpr3*cos(2*q5)*sin(q6) - qp3*qpr2*cos(2*q5)*sin(q6) - qp2*qpr4*cos(2*q5)*sin(q6) - qp3*qpr3*cos(2*q5)*sin(q6) - qp4*qpr2*cos(2*q5)*sin(q6) - qp3*qpr4*cos(2*q5)*sin(q6) - qp4*qpr3*cos(2*q5)*sin(q6) - qp4*qpr4*cos(2*q5)*sin(q6) - qp2*qpr6*cos(q5)*sin(q6) - qp6*qpr2*cos(q5)*sin(q6) - qp3*qpr6*cos(q5)*sin(q6) - qp6*qpr3*cos(q5)*sin(q6) - qp4*qpr6*cos(q5)*sin(q6) - qp6*qpr4*cos(q5)*sin(q6) - qppr1*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) - qppr1*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - qppr1*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + qppr1*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + (qp1*qpr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(q6))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(q6))/2 + (qp1*qpr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(q6))/2 - qp1*qpr2*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp2*qpr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp1*qpr3*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp3*qpr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp1*qpr4*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp4*qpr1*sin(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - qp1*qpr2*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp2*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp1*qpr3*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp3*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp1*qpr4*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - qp4*qpr1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q5) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q5)*cos(q6))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q5)*cos(q6))/2 + qp1*qpr2*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + qp1*qpr2*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) + qp1*qpr2*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp2*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + qp2*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) + qp2*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr3*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + qp1*qpr3*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) + qp1*qpr3*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp3*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + qp3*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) + qp3*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr4*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + qp1*qpr4*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) + qp1*qpr4*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp4*qpr1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q5) + qp4*qpr1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5) + qp4*qpr1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr6*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) + qp1*qpr6*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + qp1*qpr6*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) + qp6*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) + qp6*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + qp6*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) + (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5)*cos(q6))/2 - qp1*qpr6*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - qp6*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(q6))/2 + qp1*qpr2*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qp1*qpr2*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qp1*qpr2*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qp2*qpr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qp2*qpr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qp2*qpr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qp1*qpr3*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qp1*qpr3*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qp1*qpr3*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qp3*qpr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qp3*qpr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qp3*qpr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qp1*qpr4*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qp1*qpr4*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qp1*qpr4*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6) + qp4*qpr1*sin(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + qp4*qpr1*sin(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + qp4*qpr1*sin(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6);



            Yr( 5,31) =(qp2*qpr6*sin(q5))/2 + (qp6*qpr2*sin(q5))/2 + (qp3*qpr6*sin(q5))/2 + (qp6*qpr3*sin(q5))/2 + (qp4*qpr6*sin(q5))/2 + (qp6*qpr4*sin(q5))/2 + (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 + (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 + (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 + (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 + (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 + (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr6*sin(q2 + q3 + q4 - q5))/4 + (qp6*qpr1*sin(q2 + q3 + q4 - q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 - (qp1*qpr1*sin(2*q5))/4 + (qp2*qpr2*sin(2*q5))/2 + (qp2*qpr3*sin(2*q5))/2 + (qp3*qpr2*sin(2*q5))/2 + (qp2*qpr4*sin(2*q5))/2 + (qp3*qpr3*sin(2*q5))/2 + (qp4*qpr2*sin(2*q5))/2 + (qp3*qpr4*sin(2*q5))/2 + (qp4*qpr3*sin(2*q5))/2 + (qp4*qpr4*sin(2*q5))/2 + (qp1*qpr6*sin(q2 + q3 + q4 + q5))/4 + (qp6*qpr1*sin(q2 + q3 + q4 + q5))/4;



            Yr( 5,32) =qppr5 - qppr1*cos(q2 + q3 + q4) - (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr2*sin(q2 + q3 + q4))/2 + (qp2*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr3*sin(q2 + q3 + q4))/2 + (qp3*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr4*sin(q2 + q3 + q4))/2 + (qp4*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr1*sin(2*q5))/4 - (qp2*qpr2*sin(2*q5))/2 - (qp2*qpr3*sin(2*q5))/2 - (qp3*qpr2*sin(2*q5))/2 - (qp2*qpr4*sin(2*q5))/2 - (qp3*qpr3*sin(2*q5))/2 - (qp4*qpr2*sin(2*q5))/2 - (qp3*qpr4*sin(2*q5))/2 - (qp4*qpr3*sin(2*q5))/2 - (qp4*qpr4*sin(2*q5))/2;



            Yr( 5,33) =0;



            Yr( 5,34) =0;



            Yr( 5,35) =0;



            Yr( 5,36) =0;



            Yr( 5,37) =0;



            Yr( 5,38) =0;



            Yr( 5,39) =0;



            Yr( 5,40) =0;



            Yr( 5,41) =0;



            Yr( 5,42) =0;



            Yr( 5,43) =0;



            Yr( 5,44) =qppr5 - qppr1*cos(q2 + q3 + q4) - (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/4 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/8 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/8 + (qp1*qpr2*sin(q2 + q3 + q4))/2 + (qp2*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr3*sin(q2 + q3 + q4))/2 + (qp3*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr4*sin(q2 + q3 + q4))/2 + (qp4*qpr1*sin(q2 + q3 + q4))/2 + (qp1*qpr1*sin(2*q5))/4 - (qp2*qpr2*sin(2*q5))/2 - (qp2*qpr3*sin(2*q5))/2 - (qp3*qpr2*sin(2*q5))/2 - (qp2*qpr4*sin(2*q5))/2 - (qp3*qpr3*sin(2*q5))/2 - (qp4*qpr2*sin(2*q5))/2 - (qp3*qpr4*sin(2*q5))/2 - (qp4*qpr3*sin(2*q5))/2 - (qp4*qpr4*sin(2*q5))/2;



            Yr( 5,45) =0;



            Yr( 5,46) =0;



            Yr( 5,47) =0;



            Yr( 5,48) =0;



            Yr( 5,49) =0;



            Yr( 5,50) =0;



            Yr( 5,51) =0;



            Yr( 5,52) =0;



            Yr( 5,53) =0;



            Yr( 5,54) =0;



            Yr( 5,55) =sin(q1)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3);



            Yr( 5,56) =cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q1)*sin(q5) - cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4) - cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3);



            Yr( 5,57) =- sin(q2 + q3 + q4 + q5)/2 - sin(q2 + q3 + q4 - q5)/2;



            Yr( 5,58) =qp1*qpr1*sin(q5) - (qppr1*cos(q2 + q3 + q4 - q5))/2 - (qppr1*cos(q2 + q3 + q4 + q5))/2;



            Yr( 5,59) =qp1*qpr2*sin(q2)*sin(q5) - qppr1*cos(q2)*sin(q5) + qp2*qpr1*sin(q2)*sin(q5) + qppr2*cos(q3)*cos(q5)*sin(q4) + qppr2*cos(q4)*cos(q5)*sin(q3) - qp2*qpr2*cos(q3)*cos(q4)*cos(q5) + qp2*qpr2*cos(q5)*sin(q3)*sin(q4) + qp1*qpr1*pow(cos(q2),2)*cos(q5)*sin(q3)*sin(q4) - qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*cos(q5) + qp1*qpr1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp1*qpr1*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q3);



            Yr( 5,60) =2*qppr5 - 2*qppr1*cos(q2 + q3 + q4) - (qp1*qpr2*sin(q2 + q3 + q4 - 2*q5))/2 - (qp1*qpr2*sin(q2 + q3 + q4 + 2*q5))/2 - (qp2*qpr1*sin(q2 + q3 + q4 - 2*q5))/2 - (qp2*qpr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr3*sin(q2 + q3 + q4 - 2*q5))/2 - (qp1*qpr3*sin(q2 + q3 + q4 + 2*q5))/2 - (qp3*qpr1*sin(q2 + q3 + q4 - 2*q5))/2 - (qp3*qpr1*sin(q2 + q3 + q4 + 2*q5))/2 - (qp1*qpr4*sin(q2 + q3 + q4 - 2*q5))/2 - (qp1*qpr4*sin(q2 + q3 + q4 + 2*q5))/2 - (qp4*qpr1*sin(q2 + q3 + q4 - 2*q5))/2 - (qp4*qpr1*sin(q2 + q3 + q4 + 2*q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - 2*q5))/4 - (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + 2*q5))/4 + qp1*qpr2*sin(q2 + q3 + q4) + qp2*qpr1*sin(q2 + q3 + q4) + qp1*qpr3*sin(q2 + q3 + q4) + qp3*qpr1*sin(q2 + q3 + q4) + qp1*qpr4*sin(q2 + q3 + q4) + qp4*qpr1*sin(q2 + q3 + q4) + (qp1*qpr1*sin(2*q5))/2 - qp2*qpr2*sin(2*q5) - qp2*qpr3*sin(2*q5) - qp3*qpr2*sin(2*q5) - qp2*qpr4*sin(2*q5) - qp3*qpr3*sin(2*q5) - qp4*qpr2*sin(2*q5) - qp3*qpr4*sin(2*q5) - qp4*qpr3*sin(2*q5) - qp4*qpr4*sin(2*q5);



            Yr( 5,61) =qppr2*cos(q5)*sin(q4) + qppr3*cos(q5)*sin(q4) - qppr1*cos(q2)*cos(q3)*sin(q5) + qppr1*sin(q2)*sin(q3)*sin(q5) - (qp1*qpr1*cos(q4)*cos(q5))/2 - qp2*qpr2*cos(q4)*cos(q5) - qp2*qpr3*cos(q4)*cos(q5) - qp3*qpr2*cos(q4)*cos(q5) - qp3*qpr3*cos(q4)*cos(q5) + qp1*qpr2*cos(q2)*sin(q3)*sin(q5) + qp1*qpr2*cos(q3)*sin(q2)*sin(q5) + qp2*qpr1*cos(q2)*sin(q3)*sin(q5) + qp2*qpr1*cos(q3)*sin(q2)*sin(q5) + qp1*qpr3*cos(q2)*sin(q3)*sin(q5) + qp1*qpr3*cos(q3)*sin(q2)*sin(q5) + qp3*qpr1*cos(q2)*sin(q3)*sin(q5) + qp3*qpr1*cos(q3)*sin(q2)*sin(q5) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(q4)*cos(q5))/2 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*cos(q5)*sin(q4))/2 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*cos(q5)*sin(q4))/2 + (qp1*qpr1*sin(2*q2)*sin(2*q3)*cos(q4)*cos(q5))/2;



            Yr( 5,62) =(qppr1*cos(q2 + q3 + q4 - q5))/2 - qppr2*cos(q5) - qppr3*cos(q5) - qppr4*cos(q5) - (qppr1*cos(q2 + q3 + q4 + q5))/2 - (qp1*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 + (qp1*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 5,63) =qp1*qpr1*sin(q5) - (qppr1*cos(q2 + q3 + q4 - q5))/2 - (qppr1*cos(q2 + q3 + q4 + q5))/2;



            Yr( 5,64) =(qppr1*cos(q2 + q3 + q4 + q5))/2 + (qppr1*cos(q2 + q3 + q4 - q5))/2 - qp1*qpr1*sin(q5);



            Yr( 5,65) =0;



            Yr( 5,66) =0;



            Yr( 5,67) =0;



            Yr( 5,68) =0;



            Yr( 5,69) =0;



            Yr( 5,70) =0;



            Yr( 5,71) =0;



            Yr( 5,72) =0;



            Yr( 5,73) =0;



            Yr( 5,74) =0;



            Yr( 5,75) =0;



            Yr( 5,76) =sin(q1)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3);



            Yr( 5,77) =0;



            Yr( 5,78) =0;



            Yr( 5,79) =0;



            Yr( 5,80) =0;



            Yr( 5,81) =0;



            Yr( 5,82) =0;



            Yr( 5,83) =0;



            Yr( 5,84) =0;



            Yr( 5,85) =0;



            Yr( 5,86) =0;



            Yr( 5,87) =0;



            Yr( 5,88) =0;



            Yr( 5,89) =0;



            Yr( 5,90) =0;



            Yr( 5,91) =0;



            Yr( 5,92) =0;



            Yr( 5,93) =0;



            Yr( 5,94) =0;



            Yr( 5,95) =0;



            Yr( 5,96) =0;



            Yr( 5,97) =0;



            Yr( 5,98) =cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q1)*sin(q5) - cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4) - cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3);



            Yr( 5,99) =0;



            Yr( 5,100) =0;



            Yr( 5,101) =0;



            Yr( 5,102) =0;



            Yr( 5,103) =0;



            Yr( 5,104) =0;



            Yr( 5,105) =0;



            Yr( 5,106) =0;



            Yr( 5,107) =0;



            Yr( 5,108) =0;



            Yr( 5,109) =0;



            Yr( 5,110) =0;



            Yr( 5,111) =0;



            Yr( 5,112) =0;



            Yr( 5,113) =0;



            Yr( 5,114) =- sin(q2 + q3 + q4 + q5)/2 - sin(q2 + q3 + q4 - q5)/2;



            Yr( 5,115) =0;



            Yr( 5,116) =0;



            Yr( 5,117) =0;



            Yr( 5,118) =0;



            Yr( 5,119) =0;



            Yr( 5,120) =0;



            Yr( 5,121) =0;



            Yr( 5,122) =0;



            Yr( 5,123) =0;



            Yr( 5,124) =0;



            Yr( 5,125) =0;



            Yr( 5,126) =0;



            Yr( 5,127) =qp1*qpr1*sin(q5) - (qppr1*cos(q2 + q3 + q4 - q5))/2 - (qppr1*cos(q2 + q3 + q4 + q5))/2;



            Yr( 5,128) =0;



            Yr( 5,129) =0;



            Yr( 5,130) =0;



            Yr( 5,131) =qp1*qpr2*sin(q2)*sin(q5) - qppr1*cos(q2)*sin(q5) + qp2*qpr1*sin(q2)*sin(q5) + qppr2*cos(q3)*cos(q5)*sin(q4) + qppr2*cos(q4)*cos(q5)*sin(q3) - qp2*qpr2*cos(q3)*cos(q4)*cos(q5) + qp2*qpr2*cos(q5)*sin(q3)*sin(q4) + qp1*qpr1*pow(cos(q2),2)*cos(q5)*sin(q3)*sin(q4) - qp1*qpr1*pow(cos(q2),2)*cos(q3)*cos(q4)*cos(q5) + qp1*qpr1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + qp1*qpr1*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q3);



            Yr( 5,132) =0;



            Yr( 5,133) =0;



            Yr( 5,134) =0;



            Yr( 5,135) =qppr2*cos(q5)*sin(q4) + qppr3*cos(q5)*sin(q4) - qppr1*cos(q2)*cos(q3)*sin(q5) + qppr1*sin(q2)*sin(q3)*sin(q5) - (qp1*qpr1*cos(q4)*cos(q5))/2 - qp2*qpr2*cos(q4)*cos(q5) - qp2*qpr3*cos(q4)*cos(q5) - qp3*qpr2*cos(q4)*cos(q5) - qp3*qpr3*cos(q4)*cos(q5) + qp1*qpr2*cos(q2)*sin(q3)*sin(q5) + qp1*qpr2*cos(q3)*sin(q2)*sin(q5) + qp2*qpr1*cos(q2)*sin(q3)*sin(q5) + qp2*qpr1*cos(q3)*sin(q2)*sin(q5) + qp1*qpr3*cos(q2)*sin(q3)*sin(q5) + qp1*qpr3*cos(q3)*sin(q2)*sin(q5) + qp3*qpr1*cos(q2)*sin(q3)*sin(q5) + qp3*qpr1*cos(q3)*sin(q2)*sin(q5) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(q4)*cos(q5))/2 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*cos(q5)*sin(q4))/2 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*cos(q5)*sin(q4))/2 + (qp1*qpr1*sin(2*q2)*sin(2*q3)*cos(q4)*cos(q5))/2;



            Yr( 5,136) =0;



            Yr( 5,137) =0;



            Yr( 5,138) =0;



            Yr( 5,139) =0;



            Yr( 5,140) =0;



            Yr( 5,141) =0;



            Yr( 5,142) =0;



            Yr( 5,143) =0;



            Yr( 5,144) =0;



            Yr( 5,145) =0;



            Yr( 5,146) =0;



            Yr( 5,147) =(qppr1*cos(q2 + q3 + q4 - q5))/2 - qppr2*cos(q5) - qppr3*cos(q5) - qppr4*cos(q5) - (qppr1*cos(q2 + q3 + q4 + q5))/2 - (qp1*qpr2*sin(q2 + q3 + q4 - q5))/2 - (qp2*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr3*sin(q2 + q3 + q4 - q5))/2 - (qp3*qpr1*sin(q2 + q3 + q4 - q5))/2 - (qp1*qpr4*sin(q2 + q3 + q4 - q5))/2 - (qp4*qpr1*sin(q2 + q3 + q4 - q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 - q5))/4 + (qp1*qpr2*sin(q2 + q3 + q4 + q5))/2 + (qp2*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr3*sin(q2 + q3 + q4 + q5))/2 + (qp3*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr4*sin(q2 + q3 + q4 + q5))/2 + (qp4*qpr1*sin(q2 + q3 + q4 + q5))/2 + (qp1*qpr1*sin(2*q2 + 2*q3 + 2*q4 + q5))/4;



            Yr( 5,148) =0;



            Yr( 5,149) =0;



            Yr( 5,150) =0;



            Yr( 5,151) =0;



            Yr( 5,152) =0;



            Yr( 5,153) =qp1*qpr1*sin(q5) - (qppr1*cos(q2 + q3 + q4 - q5))/2 - (qppr1*cos(q2 + q3 + q4 + q5))/2;



            Yr( 5,154) =0;



            Yr( 5,155) =0;



            Yr( 5,156) =(qppr1*cos(q2 + q3 + q4 + q5))/2 + (qppr1*cos(q2 + q3 + q4 - q5))/2 - qp1*qpr1*sin(q5);



            Yr( 5,157) =0;



            Yr( 5,158) =0;



            Yr( 5,159) =0;



            Yr( 5,160) =0;



            Yr( 5,161) =0;



            Yr( 5,162) =0;



            Yr( 5,163) =0;



            Yr( 6,1) =0;



            Yr( 6,2) =0;



            Yr( 6,3) =0;



            Yr( 6,4) =0;



            Yr( 6,5) =0;



            Yr( 6,6) =0;



            Yr( 6,7) =0;



            Yr( 6,8) =0;



            Yr( 6,9) =0;



            Yr( 6,10) =0;



            Yr( 6,11) =0;



            Yr( 6,12) =0;



            Yr( 6,13) =0;



            Yr( 6,14) =0;



            Yr( 6,15) =0;



            Yr( 6,16) =0;



            Yr( 6,17) =0;



            Yr( 6,18) =0;



            Yr( 6,19) =0;



            Yr( 6,20) =0;



            Yr( 6,21) =0;



            Yr( 6,22) =0;



            Yr( 6,23) =0;



            Yr( 6,24) =0;



            Yr( 6,25) =0;



            Yr( 6,26) =(qp2*qpr2*sin(2*q6))/4 - (qp1*qpr1*sin(2*q6))/8 + (qp2*qpr3*sin(2*q6))/4 + (qp3*qpr2*sin(2*q6))/4 + (qp2*qpr4*sin(2*q6))/4 + (qp3*qpr3*sin(2*q6))/4 + (qp4*qpr2*sin(2*q6))/4 + (qp3*qpr4*sin(2*q6))/4 + (qp4*qpr3*sin(2*q6))/4 + (qp4*qpr4*sin(2*q6))/4 - (qp5*qpr5*sin(2*q6))/2 + (qp2*qpr5*cos(2*q6)*sin(q5))/2 + (qp5*qpr2*cos(2*q6)*sin(q5))/2 + (qp3*qpr5*cos(2*q6)*sin(q5))/2 + (qp5*qpr3*cos(2*q6)*sin(q5))/2 + (qp4*qpr5*cos(2*q6)*sin(q5))/2 + (qp5*qpr4*cos(2*q6)*sin(q5))/2 + (qp1*qpr1*cos(2*q5)*sin(2*q6))/8 - (qp2*qpr2*cos(2*q5)*sin(2*q6))/4 - (qp2*qpr3*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr2*cos(2*q5)*sin(2*q6))/4 - (qp2*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr3*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr2*cos(2*q5)*sin(2*q6))/4 - (qp3*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr3*cos(2*q5)*sin(2*q6))/4 - (qp4*qpr4*cos(2*q5)*sin(2*q6))/4 + (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6))/8 - (qp1*qpr5*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 - (qp1*qpr5*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (3*qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/8 + (3*qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6))/8 + (3*qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6))/8 + (qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q6))/8 + (qp1*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr2*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp2*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp2*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp2*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr3*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp3*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp3*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp3*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp1*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp1*qpr4*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp4*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 + (qp4*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 + (qp4*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 + (qp1*qpr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(2*q6))/8 + (qp1*qpr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(2*q6))/8 + (qp1*qpr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q6))/8 - (qp1*qpr2*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp2*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr3*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp3*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr4*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp4*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 - (qp1*qpr2*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp2*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qpr3*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp3*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qpr4*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp4*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2 + (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2 + (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2 + (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2 + (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*cos(q5))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*cos(q5))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*cos(q5))/2 + (qp1*qpr2*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qpr2*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qpr2*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp2*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp2*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp2*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp1*qpr3*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qpr3*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qpr3*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp3*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp3*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp3*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp1*qpr4*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp1*qpr4*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp1*qpr4*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp4*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 + (qp4*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 + (qp4*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp1*qpr5*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp5*qpr1*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 6,27) =(qp2*qpr2*cos(2*q6))/2 - (qp1*qpr1*cos(2*q6))/4 + (qp2*qpr3*cos(2*q6))/2 + (qp3*qpr2*cos(2*q6))/2 + (qp2*qpr4*cos(2*q6))/2 + (qp3*qpr3*cos(2*q6))/2 + (qp4*qpr2*cos(2*q6))/2 + (qp3*qpr4*cos(2*q6))/2 + (qp4*qpr3*cos(2*q6))/2 + (qp4*qpr4*cos(2*q6))/2 - qp5*qpr5*cos(2*q6) - qp2*qpr5*sin(2*q6)*sin(q5) - qp5*qpr2*sin(2*q6)*sin(q5) - qp3*qpr5*sin(2*q6)*sin(q5) - qp5*qpr3*sin(2*q6)*sin(q5) - qp4*qpr5*sin(2*q6)*sin(q5) - qp5*qpr4*sin(2*q6)*sin(q5) + (qp1*qpr1*cos(2*q5)*cos(2*q6))/4 - (qp2*qpr2*cos(2*q5)*cos(2*q6))/2 - (qp2*qpr3*cos(2*q5)*cos(2*q6))/2 - (qp3*qpr2*cos(2*q5)*cos(2*q6))/2 - (qp2*qpr4*cos(2*q5)*cos(2*q6))/2 - (qp3*qpr3*cos(2*q5)*cos(2*q6))/2 - (qp4*qpr2*cos(2*q5)*cos(2*q6))/2 - (qp3*qpr4*cos(2*q5)*cos(2*q6))/2 - (qp4*qpr3*cos(2*q5)*cos(2*q6))/2 - (qp4*qpr4*cos(2*q5)*cos(2*q6))/2 - (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q6))/4 - qp1*qpr5*cos(2*q6)*cos(q2)*sin(q3)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q3)*sin(q2)*sin(q4) - qp1*qpr5*cos(2*q6)*cos(q4)*sin(q2)*sin(q3) - qp5*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4) - qp5*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3) + (3*qp1*qpr1*cos(2*q2)*cos(2*q6)*sin(2*q3)*sin(2*q4))/4 + (3*qp1*qpr1*cos(2*q3)*cos(2*q6)*sin(2*q2)*sin(2*q4))/4 + (3*qp1*qpr1*cos(2*q4)*cos(2*q6)*sin(2*q2)*sin(2*q3))/4 + qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q4) + qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4) + qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5) + qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(2*q6)*cos(q5) + qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(2*q6)*cos(q5) - qp1*qpr2*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qp1*qpr2*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qp1*qpr2*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp2*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qp2*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qp2*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp1*qpr3*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qp1*qpr3*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qp1*qpr3*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp3*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qp3*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qp3*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp1*qpr4*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qp1*qpr4*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qp1*qpr4*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - qp4*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - qp4*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - qp4*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + qp1*qpr5*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + qp5*qpr1*sin(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6)*cos(q5) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*cos(2*q6))/4 + (qp1*qpr2*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr2*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr2*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp2*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 + (qp2*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 + (qp2*qpr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr3*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr3*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr3*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp3*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 + (qp3*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 + (qp3*qpr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr4*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 + (qp1*qpr4*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 + (qp1*qpr4*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp4*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q3)*sin(q4))/2 + (qp4*qpr1*cos(2*q6)*sin(2*q5)*cos(q2)*cos(q4)*sin(q3))/2 + (qp4*qpr1*cos(2*q6)*sin(2*q5)*cos(q3)*cos(q4)*sin(q2))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q5)*cos(2*q6)*sin(2*q3)*sin(2*q4))/4 + (qp1*qpr1*cos(2*q3)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q4))/4 + (qp1*qpr1*cos(2*q4)*cos(2*q5)*cos(2*q6)*sin(2*q2)*sin(2*q3))/4 - (qp1*qpr2*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp2*qpr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr3*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr4*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr1*cos(2*q6)*sin(2*q5)*sin(q2)*sin(q3)*sin(q4))/2 + qp1*qpr2*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp2*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp1*qpr3*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp3*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp1*qpr4*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + qp4*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - qp1*qpr5*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - qp1*qpr5*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2) - qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - qp5*qpr1*sin(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - qp5*qpr1*sin(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2);



            Yr( 6,28) =qppr5*sin(q6) - qppr2*cos(q6)*sin(q5) - qppr3*cos(q6)*sin(q5) - qppr4*cos(q6)*sin(q5) + (qp1*qpr1*sin(2*q5)*sin(q6))/4 - (qp2*qpr2*sin(2*q5)*sin(q6))/2 - (qp2*qpr3*sin(2*q5)*sin(q6))/2 - (qp3*qpr2*sin(2*q5)*sin(q6))/2 - (qp2*qpr4*sin(2*q5)*sin(q6))/2 - (qp3*qpr3*sin(2*q5)*sin(q6))/2 - (qp4*qpr2*sin(2*q5)*sin(q6))/2 - (qp3*qpr4*sin(2*q5)*sin(q6))/2 - (qp4*qpr3*sin(2*q5)*sin(q6))/2 - (qp4*qpr4*sin(2*q5)*sin(q6))/2 - qp2*qpr5*cos(q5)*cos(q6) - qp5*qpr2*cos(q5)*cos(q6) - qp3*qpr5*cos(q5)*cos(q6) - qp5*qpr3*cos(q5)*cos(q6) - qp4*qpr5*cos(q5)*cos(q6) - qp5*qpr4*cos(q5)*cos(q6) - qppr1*cos(q2)*cos(q3)*cos(q4)*sin(q6) + qppr1*cos(q2)*sin(q3)*sin(q4)*sin(q6) + qppr1*cos(q3)*sin(q2)*sin(q4)*sin(q6) + qppr1*cos(q4)*sin(q2)*sin(q3)*sin(q6) - qppr1*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) - qppr1*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - qppr1*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + qppr1*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + (qp1*qpr2*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr2*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp1*qpr2*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp2*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp2*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp2*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp1*qpr3*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr3*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp1*qpr3*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp3*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp3*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp3*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp1*qpr4*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr4*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp1*qpr4*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 + (qp4*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 + (qp4*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 + (qp4*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp1*qpr2*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp2*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr3*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp3*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr4*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp4*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*sin(q6))/4 + (qp1*qpr2*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp2*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr3*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp3*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr4*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp4*qpr1*cos(2*q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6))/2 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*sin(q6))/4 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*sin(q6))/4 + (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*sin(q6))/4 + qp1*qpr5*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) + qp1*qpr5*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) + qp1*qpr5*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + qp5*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q5) + qp5*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) + qp5*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) - (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*cos(q6)*sin(q5))/2 - (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*cos(q6)*sin(q5))/2 - qp1*qpr5*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - qp5*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q6)*sin(q5))/2 - (qp1*qpr2*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr2*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp1*qpr2*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp2*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp2*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp2*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp1*qpr3*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr3*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp1*qpr3*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp3*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp3*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp3*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp1*qpr4*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp1*qpr4*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp1*qpr4*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2 - (qp4*qpr1*cos(2*q5)*cos(q2)*cos(q3)*sin(q4)*sin(q6))/2 - (qp4*qpr1*cos(2*q5)*cos(q2)*cos(q4)*sin(q3)*sin(q6))/2 - (qp4*qpr1*cos(2*q5)*cos(q3)*cos(q4)*sin(q2)*sin(q6))/2;



            Yr( 6,29) =(qp1*qpr1*sin(2*q6))/8 - (qp2*qpr2*sin(2*q6))/4 - (qp2*qpr3*sin(2*q6))/4 - (qp3*qpr2*sin(2*q6))/4 - (qp2*qpr4*sin(2*q6))/4 - (qp3*qpr3*sin(2*q6))/4 - (qp4*qpr2*sin(2*q6))/4 - (qp3*qpr4*sin(2*q6))/4 - (qp4*qpr3*sin(2*q6))/4 - (qp4*qpr4*sin(2*q6))/4 + (qp5*qpr5*sin(2*q6))/2 - (qp2*qpr5*cos(2*q6)*sin(q5))/2 - (qp5*qpr2*cos(2*q6)*sin(q5))/2 - (qp3*qpr5*cos(2*q6)*sin(q5))/2 - (qp5*qpr3*cos(2*q6)*sin(q5))/2 - (qp4*qpr5*cos(2*q6)*sin(q5))/2 - (qp5*qpr4*cos(2*q6)*sin(q5))/2 - (qp1*qpr1*cos(2*q5)*sin(2*q6))/8 + (qp2*qpr2*cos(2*q5)*sin(2*q6))/4 + (qp2*qpr3*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr2*cos(2*q5)*sin(2*q6))/4 + (qp2*qpr4*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr3*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr2*cos(2*q5)*sin(2*q6))/4 + (qp3*qpr4*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr3*cos(2*q5)*sin(2*q6))/4 + (qp4*qpr4*cos(2*q5)*sin(2*q6))/4 - (qp1*qpr5*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 - (qp5*qpr1*sin(2*q6)*cos(q2)*cos(q3)*cos(q4))/2 + (3*qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q6))/8 + (qp1*qpr5*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp1*qpr5*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 + (qp5*qpr1*sin(2*q6)*cos(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q3)*sin(q2)*sin(q4))/2 + (qp5*qpr1*sin(2*q6)*cos(q4)*sin(q2)*sin(q3))/2 - (3*qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q6))/8 - (3*qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q6))/8 - (3*qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q6))/8 - (qp1*qpr1*cos(2*q6)*sin(2*q2)*sin(2*q3)*sin(2*q4)*cos(q5))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*cos(2*q5)*sin(2*q6))/8 - (qp1*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr2*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr2*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp2*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp2*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp2*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr3*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr3*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp3*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp3*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp3*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp1*qpr4*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp1*qpr4*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp4*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q3)*sin(q4))/4 - (qp4*qpr1*sin(2*q5)*sin(2*q6)*cos(q2)*cos(q4)*sin(q3))/4 - (qp4*qpr1*sin(2*q5)*sin(2*q6)*cos(q3)*cos(q4)*sin(q2))/4 - (qp1*qpr1*cos(2*q2)*cos(2*q5)*sin(2*q3)*sin(2*q4)*sin(2*q6))/8 - (qp1*qpr1*cos(2*q3)*cos(2*q5)*sin(2*q2)*sin(2*q4)*sin(2*q6))/8 - (qp1*qpr1*cos(2*q4)*cos(2*q5)*sin(2*q2)*sin(2*q3)*sin(2*q6))/8 + (qp1*qpr2*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp2*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr3*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp3*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr4*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp4*qpr1*sin(2*q5)*sin(2*q6)*sin(q2)*sin(q3)*sin(q4))/4 + (qp1*qpr2*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp2*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp1*qpr3*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp3*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp1*qpr4*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 + (qp4*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2 - (qp1*qpr5*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2 - (qp1*qpr5*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2 - (qp5*qpr1*cos(2*q6)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2 - (qp5*qpr1*cos(2*q6)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q6)*sin(2*q4)*cos(q5))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*cos(2*q6)*sin(2*q3)*cos(q5))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*cos(2*q6)*sin(2*q2)*cos(q5))/2 - (qp1*qpr2*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp1*qpr2*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp1*qpr2*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp2*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp2*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp2*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp1*qpr3*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp1*qpr3*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp1*qpr3*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp3*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp3*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp3*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp1*qpr4*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp1*qpr4*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp1*qpr4*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 - (qp4*qpr1*cos(2*q6)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2 - (qp4*qpr1*cos(2*q6)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2 - (qp4*qpr1*cos(2*q6)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2 + (qp1*qpr5*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2 + (qp5*qpr1*cos(2*q6)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 6,30) =qppr5*cos(q6) + qppr2*sin(q5)*sin(q6) + qppr3*sin(q5)*sin(q6) + qppr4*sin(q5)*sin(q6) + (qp1*qpr1*sin(2*q5)*cos(q6))/4 - (qp2*qpr2*sin(2*q5)*cos(q6))/2 - (qp2*qpr3*sin(2*q5)*cos(q6))/2 - (qp3*qpr2*sin(2*q5)*cos(q6))/2 - (qp2*qpr4*sin(2*q5)*cos(q6))/2 - (qp3*qpr3*sin(2*q5)*cos(q6))/2 - (qp4*qpr2*sin(2*q5)*cos(q6))/2 - (qp3*qpr4*sin(2*q5)*cos(q6))/2 - (qp4*qpr3*sin(2*q5)*cos(q6))/2 - (qp4*qpr4*sin(2*q5)*cos(q6))/2 + qp2*qpr5*cos(q5)*sin(q6) + qp5*qpr2*cos(q5)*sin(q6) + qp3*qpr5*cos(q5)*sin(q6) + qp5*qpr3*cos(q5)*sin(q6) + qp4*qpr5*cos(q5)*sin(q6) + qp5*qpr4*cos(q5)*sin(q6) - qppr1*cos(q2)*cos(q3)*cos(q4)*cos(q6) + qppr1*cos(q2)*cos(q6)*sin(q3)*sin(q4) + qppr1*cos(q3)*cos(q6)*sin(q2)*sin(q4) + qppr1*cos(q4)*cos(q6)*sin(q2)*sin(q3) + qppr1*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + qppr1*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + qppr1*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) - qppr1*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + (qp1*qpr2*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp1*qpr2*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp1*qpr2*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp2*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp2*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp2*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp1*qpr3*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp1*qpr3*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp1*qpr3*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp3*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp3*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp3*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp1*qpr4*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp1*qpr4*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp1*qpr4*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 + (qp4*qpr1*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 + (qp4*qpr1*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 + (qp4*qpr1*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp1*qpr2*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp2*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr3*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp3*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp1*qpr4*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 - (qp4*qpr1*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr1*cos(2*q2)*sin(2*q3)*sin(2*q4)*sin(2*q5)*cos(q6))/4 + (qp1*qpr1*cos(2*q3)*sin(2*q2)*sin(2*q4)*sin(2*q5)*cos(q6))/4 + (qp1*qpr1*cos(2*q4)*sin(2*q2)*sin(2*q3)*sin(2*q5)*cos(q6))/4 - qp1*qpr5*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) - qp1*qpr5*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) - qp1*qpr5*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) - qp5*qpr1*cos(q2)*cos(q3)*sin(q4)*sin(q5)*sin(q6) - qp5*qpr1*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) - qp5*qpr1*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6) + (qp1*qpr1*cos(2*q2)*cos(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q2)*cos(2*q4)*sin(2*q3)*sin(q5)*sin(q6))/2 + (qp1*qpr1*cos(2*q3)*cos(2*q4)*sin(2*q2)*sin(q5)*sin(q6))/2 + qp1*qpr5*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) + qp5*qpr1*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6) - (qp1*qpr1*sin(2*q2)*sin(2*q3)*sin(2*q4)*sin(q5)*sin(q6))/2 - (qp1*qpr2*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp1*qpr2*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp1*qpr2*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp2*qpr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp2*qpr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp2*qpr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp1*qpr3*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp1*qpr3*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp1*qpr3*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp3*qpr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp3*qpr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp3*qpr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp1*qpr4*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp1*qpr4*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp1*qpr4*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp4*qpr1*cos(2*q5)*cos(q2)*cos(q3)*cos(q6)*sin(q4))/2 - (qp4*qpr1*cos(2*q5)*cos(q2)*cos(q4)*cos(q6)*sin(q3))/2 - (qp4*qpr1*cos(2*q5)*cos(q3)*cos(q4)*cos(q6)*sin(q2))/2 - (qp1*qpr1*cos(2*q2)*cos(2*q3)*cos(2*q4)*sin(2*q5)*cos(q6))/4 + (qp1*qpr2*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp2*qpr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr3*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp3*qpr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp1*qpr4*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2 + (qp4*qpr1*cos(2*q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4))/2;



            Yr( 6,31) =qppr6 + (qppr1*cos(q2 + q3 + q4 + q5))/2 + qppr2*cos(q5) + qppr3*cos(q5) + qppr4*cos(q5) - (qppr1*cos(q2 + q3 + q4 - q5))/2 - (qp2*qpr5*sin(q5))/2 - (qp5*qpr2*sin(q5))/2 - (qp3*qpr5*sin(q5))/2 - (qp5*qpr3*sin(q5))/2 - (qp4*qpr5*sin(q5))/2 - (qp5*qpr4*sin(q5))/2 + (qp1*qpr2*sin(q2 + q3 + q4 - q5))/4 + (qp2*qpr1*sin(q2 + q3 + q4 - q5))/4 + (qp1*qpr3*sin(q2 + q3 + q4 - q5))/4 + (qp3*qpr1*sin(q2 + q3 + q4 - q5))/4 + (qp1*qpr4*sin(q2 + q3 + q4 - q5))/4 + (qp4*qpr1*sin(q2 + q3 + q4 - q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 - q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 - q5))/4 - (qp1*qpr2*sin(q2 + q3 + q4 + q5))/4 - (qp2*qpr1*sin(q2 + q3 + q4 + q5))/4 - (qp1*qpr3*sin(q2 + q3 + q4 + q5))/4 - (qp3*qpr1*sin(q2 + q3 + q4 + q5))/4 - (qp1*qpr4*sin(q2 + q3 + q4 + q5))/4 - (qp4*qpr1*sin(q2 + q3 + q4 + q5))/4 - (qp1*qpr5*sin(q2 + q3 + q4 + q5))/4 - (qp5*qpr1*sin(q2 + q3 + q4 + q5))/4;



            Yr( 6,32) =0;



            Yr( 6,33) =0;



            Yr( 6,34) =0;



            Yr( 6,35) =0;



            Yr( 6,36) =0;



            Yr( 6,37) =0;



            Yr( 6,38) =0;



            Yr( 6,39) =0;



            Yr( 6,40) =0;



            Yr( 6,41) =0;



            Yr( 6,42) =0;



            Yr( 6,43) =0;



            Yr( 6,44) =0;



            Yr( 6,45) =0;



            Yr( 6,46) =0;



            Yr( 6,47) =0;



            Yr( 6,48) =0;



            Yr( 6,49) =0;



            Yr( 6,50) =0;



            Yr( 6,51) =0;



            Yr( 6,52) =0;



            Yr( 6,53) =0;



            Yr( 6,54) =0;



            Yr( 6,55) =0;



            Yr( 6,56) =0;



            Yr( 6,57) =0;



            Yr( 6,58) =0;



            Yr( 6,59) =0;



            Yr( 6,60) =0;



            Yr( 6,61) =0;



            Yr( 6,62) =0;



            Yr( 6,63) =0;



            Yr( 6,64) =0;



            Yr( 6,65) =0;



            Yr( 6,66) =0;



            Yr( 6,67) =0;



            Yr( 6,68) =0;



            Yr( 6,69) =0;



            Yr( 6,70) =0;



            Yr( 6,71) =0;



            Yr( 6,72) =0;



            Yr( 6,73) =0;



            Yr( 6,74) =0;



            Yr( 6,75) =0;



            Yr( 6,76) =0;



            Yr( 6,77) =0;



            Yr( 6,78) =0;



            Yr( 6,79) =0;



            Yr( 6,80) =0;



            Yr( 6,81) =0;



            Yr( 6,82) =0;



            Yr( 6,83) =0;



            Yr( 6,84) =0;



            Yr( 6,85) =0;



            Yr( 6,86) =0;



            Yr( 6,87) =0;



            Yr( 6,88) =0;



            Yr( 6,89) =0;



            Yr( 6,90) =0;



            Yr( 6,91) =0;



            Yr( 6,92) =0;



            Yr( 6,93) =0;



            Yr( 6,94) =0;



            Yr( 6,95) =0;



            Yr( 6,96) =0;



            Yr( 6,97) =0;



            Yr( 6,98) =0;



            Yr( 6,99) =0;



            Yr( 6,100) =0;



            Yr( 6,101) =0;



            Yr( 6,102) =0;



            Yr( 6,103) =0;



            Yr( 6,104) =0;



            Yr( 6,105) =0;



            Yr( 6,106) =0;



            Yr( 6,107) =0;



            Yr( 6,108) =0;



            Yr( 6,109) =0;



            Yr( 6,110) =0;



            Yr( 6,111) =0;



            Yr( 6,112) =0;



            Yr( 6,113) =0;



            Yr( 6,114) =0;



            Yr( 6,115) =0;



            Yr( 6,116) =0;



            Yr( 6,117) =0;



            Yr( 6,118) =0;



            Yr( 6,119) =0;



            Yr( 6,120) =0;



            Yr( 6,121) =0;



            Yr( 6,122) =0;



            Yr( 6,123) =0;



            Yr( 6,124) =0;



            Yr( 6,125) =0;



            Yr( 6,126) =0;



            Yr( 6,127) =0;



            Yr( 6,128) =0;



            Yr( 6,129) =0;



            Yr( 6,130) =0;



            Yr( 6,131) =0;



            Yr( 6,132) =0;



            Yr( 6,133) =0;



            Yr( 6,134) =0;



            Yr( 6,135) =0;



            Yr( 6,136) =0;



            Yr( 6,137) =0;



            Yr( 6,138) =0;



            Yr( 6,139) =0;



            Yr( 6,140) =0;



            Yr( 6,141) =0;



            Yr( 6,142) =0;



            Yr( 6,143) =0;



            Yr( 6,144) =0;



            Yr( 6,145) =0;



            Yr( 6,146) =0;



            Yr( 6,147) =0;



            Yr( 6,148) =0;



            Yr( 6,149) =0;



            Yr( 6,150) =0;



            Yr( 6,151) =0;



            Yr( 6,152) =0;



            Yr( 6,153) =0;



            Yr( 6,154) =0;



            Yr( 6,155) =0;



            Yr( 6,156) =0;



            Yr( 6,157) =0;



            Yr( 6,158) =0;



            Yr( 6,159) =0;



            Yr( 6,160) =0;



            Yr( 6,161) =0;



            Yr( 6,162) =0;



            Yr( 6,163) =0;







        // remap the matrix to its real size.
        Eigen::MatrixXd Yr_ = Yr.block(1, 1, 6, 163);
        return Yr_;

        }

        Eigen::MatrixXd get_parameters(){

            Eigen::MatrixXd params_(164, 1);
            params_.setZero();
            /* Use advanced Eigen initialisation to feed the saved symbolic values from python saved file */
            // remap the matrix to its real size.

            return params_;

        }


    }

}