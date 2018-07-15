/*!
    \file utils/Interface.cc
    \brief Grasp interface

    \author João Borrego : jsbruglie
*/

#include "Interface.hh"

//////////////////////////////////////////////////
Interface::Interface()
{
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    pub = node->Advertise<grasp::msgs::Hand>(HAND_PLUGIN_TOPIC);
    pub->WaitForConnection();

    std::vector<double> values;

    joints = {
        "virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
        "virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint",
        "rh_FFJ4","rh_FFJ3","rh_FFJ2",
        "rh_MFJ4","rh_MFJ3","rh_MFJ2",
        "rh_RFJ4","rh_RFJ3","rh_RFJ2",
        "rh_LFJ5","rh_LFJ4","rh_LFJ3","rh_LFJ2",
        "rh_THJ5","rh_THJ4","rh_THJ3","rh_THJ2"
    };
    values = {
        0.0, 0.0, 0.5,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    };

    grasp::msgs::Hand msg;
    for (unsigned int i = 0; i < joints.size(); i++)
    {    
        const char *joint = joints.at(i).c_str();
        state[joint] = 0.0;
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(POSITION);
        target->set_joint(joint);
        target->set_value(0.0);
    }
    pub->Publish(msg);
}

//////////////////////////////////////////////////
void Interface::loop()
{
    bool loop = true;
    char key = -1;

    while (loop)
    {
        // Keyboard input
        if (kbhit()) {
            key = getchar();
            if (key == KEY_EXIT) {
                loop = false;
            } else {
                processKeypress(key);
            }
        }    
    }

    return;
}

//////////////////////////////////////////////////
int Interface::processKeypress(char key)
{
    switch (key)
    {
        // Translate base
        case 'w':
            moveJoint("virtual_px_joint",  0.05); break;
        case 's':
            moveJoint("virtual_px_joint", -0.05); break;
        case 'a':
            moveJoint("virtual_py_joint",  0.05); break;
        case 'd':
            moveJoint("virtual_py_joint", -0.05); break;
        case 'e':
            moveJoint("virtual_pz_joint",  0.05); break;
        case 'q':
            moveJoint("virtual_pz_joint", -0.05); break;
        // Rotate base
        case 'i':
            moveJoint("virtual_rr_joint",  0.025); break;
        case 'k':
            moveJoint("virtual_rr_joint", -0.025); break;
        case 'j':
            moveJoint("virtual_rp_joint",  0.025); break;
        case 'l':
            moveJoint("virtual_rp_joint", -0.025); break;
        case 'u':
            moveJoint("virtual_ry_joint",  0.025); break;
        case 'o':
            moveJoint("virtual_ry_joint", -0.025); break;
        default:
            break;
    }
    return 0;
}

//////////////////////////////////////////////////
void Interface::moveJoint(const char *joint, float value)
{ 
    auto search = state.find(joint);
    if (search != state.end())
    {
        search->second += value;
        grasp::msgs::Hand msg;
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(POSITION);
        target->set_joint(joint);
        target->set_value(search->second);
        pub->Publish(msg);
    }
}

//////////////////////////////////////////////////
void Interface::moveJoints(
    std::vector<std::string> & joints,
    std::vector<float> & values)
{ 
    if (joints.size() == values.size())
    {
        grasp::msgs::Hand msg;
        for (unsigned int i = 0; i < joints.size(); i++)
        {
            const char *joint = joints.at(i).c_str();
            state[joint] = 0.0;
            grasp::msgs::Target *target = msg.add_pid_targets();
            target->set_type(POSITION);
            target->set_joint(joint);
            target->set_value(values.at(i));
        }
        pub->Publish(msg);
    }
}
