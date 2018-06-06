/*!
    \file utils/contacts_example.cc
    \brief TODO

    TODO

    \author João Borrego : jsbruglie
*/

#include "contacts_example.hh"

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the target plugin request topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<grasp::msgs::ContactSensorRequest>(REQUEST_TOPIC);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Main loop
    std::string line = "";
    while (std::cout << PROMPT)
    {
        // Process command
        getline(std::cin, line);
        std::stringstream input_stream(line);
        std::string command = input_stream.str();
        std::string col1("ground_plane"), col2("box");
        getContactBetween(pub, col1, col2);
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

/////////////////////////////////////////////////
void getContactBetween(gazebo::transport::PublisherPtr pub,
    const std::string & collision1,
    const std::string & collision2)
{
    ContactSensorRequest msg;
    msg.set_type(REQ_IN_CONTACT);
    msg.set_in_contact(collision1);
    pub->Publish(msg);
}