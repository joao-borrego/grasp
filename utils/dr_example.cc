/*!
    \file utils/dr_example.cc
    \brief Domain randomization example client
    \author João Borrego : jsbruglie
*/

#include "dr_example.hh"

// Reduce verbosity
using namespace ignition::math;

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    DRInterface interface;
    DRRequest msg;

    std::string model("baxter_gripper");
    std::string joint("gripper_l_finger_joint");
    std::string joint_scoped = model + "::" + joint;
    std::string link("gripper_l_finger");
    int pid_type = DRInterface::POSITION;
    double inf = INFINITY;

    // Disable gravity
    interface.addGravity(msg, Vector3d(0,0,0));
    // Rescale baxter hand
    interface.addModelScale(msg, model, Vector3d(2,2,2) );
    // Update link mass
    interface.addLinkMass(msg, model, link, 200);
    // Update joint properties
    interface.addJoint(msg, model, joint, -1, 1, 10,10, inf, 200); 
    // Update PID controller
    interface.addModelCmd(msg, model, joint_scoped, pid_type, 1000, inf, inf);

    interface.publish(msg);

    // Shut down
    gazebo::client::shutdown();
    return 0;
}
