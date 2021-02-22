#include <robot_init.h>
#include <math.h>

using namespace std;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    auto robot = ecn::initRobot(argc, argv, 100);
    const unsigned n = robot->getDofs();

    // robot properties
    const vpColVector vMax = robot->vMax();
    const vpColVector aMax = robot->aMax();

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity

    // TODO declare other variables if needed
    vpColVector q0(n), qf(n);        // joint position setpoint for initial and final poses
    double t, t0, tf;

    // main control loop
    while(robot->ok())
    {
        // current time
        t = robot->time();

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            t0 = t;
        }


        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ecn::Robot::MODE_POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            // TODO: fill the fMw function
            robot->checkPose(M);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_MANUAL)
        {
            robot->checkPose(M);
            // follow a given operational velocity
            v = robot->vw();

            // TODO: fill the fJw function
            // TODO: compute vCommand

            vpMatrix fRe = M.getRotationMatrix();
            vpMatrix R_2(6,6);

            ecn::putAt(R_2, fRe, 0, 0);
            ecn::putAt(R_2, fRe, 3, 3);

            vpMatrix J_inv = robot->fJe(q).pseudoInverse();
            vCommand = J_inv*R_2*v;


            robot->setJointVelocity(vCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            // TODO: fill the inverseGeometry function

            qf = robot->inverseGeometry(Md, q);
            std::cout << "IG: " << qf << std::endl;
            robot->setJointPosition(qf);
        }




        else if(robot->mode() == ecn::Robot::MODE_INTERP_P2P)
        {
            // reach Md with interpolated joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);
            }

            // TODO: compute qCommand from q0, qf, t, t0 and tf

            vpColVector tf_vec(n);

            for (int i = 0; i<int(n); i++)
            {
                auto dq = qf[i] - q0[i];
                auto tf_v = 15*(abs(dq))/(8*vMax[i]);
                auto tf_a = sqrt(10*(abs(dq))/(sqrt(3)*aMax[i]));
                //auto tf_v = 1.5*abs(dq)/vMax[i];
                //auto tf_a = sqrt(6*abs(dq)/aMax[i]);
                tf_vec[i] = max(tf_v, tf_a);
            }
            tf = tf_vec.getMaxValue();
            auto pt = 10*pow((t-t0)/tf,3) - 15*pow((t-t0)/tf,4) + 6*pow((t-t0)/tf,5);
            //auto pt = 3*pow((t-t0/tf), 2) - 2*pow((t-t0)/tf)
            qCommand = q0 + pt*(qf - q0);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_STRAIGHT_LINE_P2P)
        {
            // go from M0 to Md in 1 sec
            tf = 1;

            // TODO: compute qCommand from M0, Md, t, t0 and tf
            // use robot->intermediaryPose to build poses between M0 and Md


            double alpha = (t-t0)/tf;
            std::cout << "alpha: " << alpha << std::endl;
            auto M_inter = robot->intermediaryPose(M0,Md,alpha);
            qCommand = robot->inverseGeometry(M_inter, q);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_P2P)
        {
            // go to Md using operational velocity

            // TODO: compute joint velocity command
            //pd - error between desired pose at current
            pd.buildFrom(M.inverse()*Md);
            v.resize(6, false);
            ecn::putAt(v, M.getRotationMatrix()*pd.getTranslationVector(), 0);
            ecn::putAt(v, M.getRotationMatrix()*(vpColVector)pd.getThetaUVector(), 3);
            vCommand = robot->lambda()*robot->fJe(q).pseudoInverse()*v;
            robot->setJointVelocity(vCommand);
        }


    }
}
