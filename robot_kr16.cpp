#include <robot_kr16.h>
#include <trig_solvers.h>
#include <math.h>

// Model of Kuka KR16 robot



// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{

    const double r6 = 0.158;

    //Generated end-effector code
    wMe[0][0] = 1.;
    wMe[0][1] = 0;
    wMe[0][2] = 0;
    wMe[0][3] = 0;
    wMe[1][0] = 0;
    wMe[1][1] = -1.;
    wMe[1][2] = 0;
    wMe[1][3] = 0;
    wMe[2][0] = 0;
    wMe[2][1] = 0;
    wMe[2][2] = -1.;
    wMe[2][3] = -r6;
    wMe[3][0] = 0;
    wMe[3][1] = 0;
    wMe[3][2] = 0;
    wMe[3][3] = 1.;
    //End of end-effector code

}


// Direct Geometry
vpHomogeneousMatrix ecn::RobotKr16::fMw(const vpColVector &q) const
{
    const double r1 = 0.675;
    const double r2 = 0.26;
    const double r3 = 0.68;
    const double r4 = 0.035;
    const double r5 = 0.67;

    vpHomogeneousMatrix M;

    // Generated pose code
    const double c1 = cos(q[0]);
    const double c2 = cos(q[1]);
    const double c4 = cos(q[3]);
    const double c5 = cos(q[4]);
    const double c6 = cos(q[5]);
    const double c23 = cos(q[1]+q[2]);
    const double s1 = sin(q[0]);
    const double s2 = sin(q[1]);
    const double s4 = sin(q[3]);
    const double s5 = sin(q[4]);
    const double s6 = sin(q[5]);
    const double s23 = sin(q[1]+q[2]);
    M[0][0] = (-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*c6 - (s1*c4 - s4*s23*c1)*s6;
    M[0][1] = -(-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*s6 - (s1*c4 - s4*s23*c1)*c6;
    M[0][2] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
    M[0][3] = (r2 + r3*c2 - r4*s23 + r5*c23)*c1;
    M[1][0] = ((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*c6 - (s1*s4*s23 + c1*c4)*s6;
    M[1][1] = -((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*s6 - (s1*s4*s23 + c1*c4)*c6;
    M[1][2] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
    M[1][3] = (-r2 - r3*c2 + r4*s23 - r5*c23)*s1;
    M[2][0] = (s5*s23 - c4*c5*c23)*c6 + s4*s6*c23;
    M[2][1] = -(s5*s23 - c4*c5*c23)*s6 + s4*c6*c23;
    M[2][2] = s5*c4*c23 + s23*c5;
    M[2][3] = r1 - r3*s2 - r4*c23 - r5*s23;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1.;
    // End of pose code

    return M;
}


// Inverse Geometry
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{

    const double r1 = 0.675;
    const double r2 = 0.26;
    const double r3 = 0.68;
    const double r4 = 0.035;
    const double r5 = 0.67;
    // desired wrist pose
    vpHomogeneousMatrix fMw = Md * wMe.inverse();

    auto tx = fMw[0][3];
    auto ty = fMw[1][3];
    auto tz = fMw[2][3];


    for (auto q2_q23:solveType7(r4, r5, tz-r1,(sqrt(sqr(tx) + sqr(ty)))-r2, 0, -r3))
    {

         auto q23 = q2_q23.qi;   // extract joint i = q23
         auto q2 = q2_q23.qj;   // extract joint j = q2
         auto q3 = q23 - q2;
         double q1 = atan2(-ty, tx);

             vpHomogeneousMatrix fM3;

             // Generated pose code
             const double c1 = cos(q1);
             const double c2 = cos(q2);
             const double c23 = cos(q2+q3);
             const double s1 = sin(q1);
             const double s2 = sin(q2);
             const double s23 = sin(q2+q3);
             fM3[0][0] = -s23*c1;
             fM3[0][1] = -c1*c23;
             fM3[0][2] = s1;
             fM3[0][3] = (r2 + r3*c2)*c1;
             fM3[1][0] = s1*s23;
             fM3[1][1] = s1*c23;
             fM3[1][2] = c1;
             fM3[1][3] = -(r2 + r3*c2)*s1;
             fM3[2][0] = -c23;
             fM3[2][1] = s23;
             fM3[2][2] = 0;
             fM3[2][3] = r1 - r3*s2;
             fM3[3][0] = 0;
             fM3[3][1] = 0;
             fM3[3][2] = 0;
             fM3[3][3] = 1.;
             // End of pose code


             auto fRw = fMw.getRotationMatrix();
             auto fR3 = fM3.getRotationMatrix();

             vpRotationMatrix R3w;;
             R3w = fR3.inverse() * fRw;

             double q5 = acos(R3w[1][2]);

             for (auto q6: solveType2(-sin(q5),sin(q5),R3w[1][1]+R3w[1][0]))
             {

                 for (auto q4: solveType3(-cos(q6),-sin(q6)*cos(q5),R3w[0][1], sin(q6)*cos(q5),-cos(q6), R3w[2][1]))
                 {
                     addCandidate({q1, q2, q3, q4, q5, q6});
                 }
             }

    }
    auto best = bestCandidate(q0);
    std::cout << "bestCandidate(q0): " << bestCandidate(q0) << std::endl;
    std::cout << "best: " << best << std::endl;
    return best;

}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);

    const double r2 = 0.26;
    const double r3 = 0.68;
    const double r4 = 0.035;
    const double r5 = 0.67;

    // Generated Jacobian code
    const double c1 = cos(q[0]);
    const double c2 = cos(q[1]);
    const double c4 = cos(q[3]);
    const double c5 = cos(q[4]);
    const double c23 = cos(q[1]+q[2]);
    const double s1 = sin(q[0]);
    const double s2 = sin(q[1]);
    const double s4 = sin(q[3]);
    const double s5 = sin(q[4]);
    const double s23 = sin(q[1]+q[2]);
    J[0][0] = (-r2 - r3*c2 + r4*s23 - r5*c23)*s1;
    J[0][1] = -(r3*s2 + r4*c23 + r5*s23)*c1;
    J[0][2] = -(r4*c23 + r5*s23)*c1;
    //J[0][3] = 0;
    //J[0][4] = 0;
    //J[0][5] = 0;
    J[1][0] = -(r2 + r3*c2 - r4*s23 + r5*c23)*c1;
    J[1][1] = (r3*s2 + r4*c23 + r5*s23)*s1;
    J[1][2] = (r4*c23 + r5*s23)*s1;
    //J[1][3] = 0;
    //J[1][4] = 0;
    //J[1][5] = 0;
    //J[2][0] = 0;
    J[2][1] = -r3*c2 + r4*s23 - r5*c23;
    J[2][2] = r4*s23 - r5*c23;
    //J[2][3] = 0;
    //J[2][4] = 0;
    //J[2][5] = 0;
    //J[3][0] = 0;
    J[3][1] = s1;
    J[3][2] = s1;
    J[3][3] = -c1*c23;
    J[3][4] = s1*c4 - s4*s23*c1;
    J[3][5] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
    //J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = c1;
    J[4][3] = s1*c23;
    J[4][4] = s1*s4*s23 + c1*c4;
    J[4][5] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
    J[5][0] = -1.;
    //J[5][1] = 0;
    //J[5][2] = 0;
    J[5][3] = s23;
    J[5][4] = -s4*c23;
    J[5][5] = s5*c4*c23 + s23*c5;
    // End of Jacobian code

     return J;


}
