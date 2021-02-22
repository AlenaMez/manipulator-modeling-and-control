#include <robot_ur10.h>
#include <trig_solvers.h>

// Model of UR-10 robot

// Any constant transform at base of end-effector
void ecn::RobotUR10::init_wMe()
{

    const double r6 = 0.1922;
    // Generated end-effector code
        wMe[0][0] = 1.;
        wMe[0][1] = 0;
        wMe[0][2] = 0;
        wMe[0][3] = 0;
        wMe[1][0] = 0;
        wMe[1][1] = 1.;
        wMe[1][2] = 0;
        wMe[1][3] = 0;
        wMe[2][0] = 0;
        wMe[2][1] = 0;
        wMe[2][2] = 1.;
        wMe[2][3] = r6;
        wMe[3][0] = 0;
        wMe[3][1] = 0;
        wMe[3][2] = 0;
        wMe[3][3] = 1.;
    // End of end-effector code

}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotUR10::fMw(const vpColVector &q) const
{
    const double r1 = 0.1273;
    const double r2 = 0.612;
    const double r3 = 0.5723;
    const double r4 = 0.1639;
    const double r5 = 0.1157;


    vpHomogeneousMatrix M;


    // Generated pose code
        const double c1 = cos(q[0]);
        const double c2 = cos(q[1]);
        const double c5 = cos(q[4]);
        const double c6 = cos(q[5]);
        const double c23 = cos(q[1]+q[2]);
        const double c234 = cos(q[1]+q[2]+q[3]);
        const double s1 = sin(q[0]);
        const double s2 = sin(q[1]);
        const double s5 = sin(q[4]);
        const double s6 = sin(q[5]);
        const double s23 = sin(q[1]+q[2]);
        const double s234 = sin(q[1]+q[2]+q[3]);
        M[0][0] = -(s1*s5 + c1*c5*c234)*c6 + s6*s234*c1;
        M[0][1] = (s1*s5 + c1*c5*c234)*s6 + s234*c1*c6;
        M[0][2] = -s1*c5 + s5*c1*c234;
        M[0][3] = r2*c1*c2 + r3*c1*c23 - r4*s1 - r5*s234*c1;
        M[1][0] = (-s1*c5*c234 + s5*c1)*c6 + s1*s6*s234;
        M[1][1] = -(-s1*c5*c234 + s5*c1)*s6 + s1*s234*c6;
        M[1][2] = s1*s5*c234 + c1*c5;
        M[1][3] = r2*s1*c2 + r3*s1*c23 + r4*c1 - r5*s1*s234;
        M[2][0] = s6*c234 + s234*c5*c6;
        M[2][1] = -s6*s234*c5 + c6*c234;
        M[2][2] = -s5*s234;
        M[2][3] = r1 - r2*s2 - r3*s23 - r5*c234;
        M[3][0] = 0;
        M[3][1] = 0;
        M[3][2] = 0;
        M[3][3] = 1.;
        // End of pose code


    return M;
}


// Inverse Geometry is already given for this robot


// Wrist Jacobian
vpMatrix ecn::RobotUR10::fJw(const vpColVector &q) const
{

    const double r2 = 0.612;
    const double r3 = 0.5723;
    const double r4 = 0.1639;
    const double r5 = 0.1157;

    vpMatrix J(6, dofs);

    // Generated Jacobian code
       const double c1 = cos(q[0]);
       const double c2 = cos(q[1]);
       const double c5 = cos(q[4]);
       const double c23 = cos(q[1]+q[2]);
       const double c234 = cos(q[1]+q[2]+q[3]);
       const double s1 = sin(q[0]);
       const double s2 = sin(q[1]);
       const double s5 = sin(q[4]);
       const double s23 = sin(q[1]+q[2]);
       const double s234 = sin(q[1]+q[2]+q[3]);
       J[0][0] = -r2*s1*c2 - r3*s1*c23 - r4*c1 + r5*s1*s234;
       J[0][1] = -(r2*s2 + r3*s23 + r5*c234)*c1;
       J[0][2] = -(r3*s23 + r5*c234)*c1;
       J[0][3] = -r5*c1*c234;
       //J[0][4] = 0;
       //J[0][5] = 0;
       J[1][0] = r2*c1*c2 + r3*c1*c23 - r4*s1 - r5*s234*c1;
       J[1][1] = -(r2*s2 + r3*s23 + r5*c234)*s1;
       J[1][2] = -(r3*s23 + r5*c234)*s1;
       J[1][3] = -r5*s1*c234;
       //J[1][4] = 0;
       //J[1][5] = 0;
       //J[2][0] = 0;
       J[2][1] = -r2*c2 - r3*c23 + r5*s234;
       J[2][2] = -r3*c23 + r5*s234;
       J[2][3] = r5*s234;
       //J[2][4] = 0;
       //J[2][5] = 0;
       //J[3][0] = 0;
       J[3][1] = -s1;
       J[3][2] = -s1;
       J[3][3] = -s1;
       J[3][4] = -s234*c1;
       J[3][5] = -s1*c5 + s5*c1*c234;
       //J[4][0] = 0;
       J[4][1] = c1;
       J[4][2] = c1;
       J[4][3] = c1;
       J[4][4] = -s1*s234;
       J[4][5] = s1*s5*c234 + c1*c5;
       J[5][0] = 1.;
       //J[5][1] = 0;
       //J[5][2] = 0;
       //J[5][3] = 0;
       J[5][4] = -c234;
       J[5][5] = -s5*s234;
       // End of Jacobian code

    return J;
}
