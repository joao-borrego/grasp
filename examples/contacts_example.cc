/*!
    \file examples/contacts_example.cc
    \brief Contact world plugin example

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
        node->Advertise<grasp::msgs::ContactRequest>(REQUEST_TOPIC);

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

        // Check for collision between box and ground plane
        if (command == "check")
        {
            std::string col1("ground_plane"), col2("box");
            getContactBetween(pub, col1, col2);
        }
        else if (command == "change")
        {
            std::string model("box"), link("link"),
                collision("box_collision");
            changeSurface(pub, model, link, collision);
        }
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
    ContactRequest msg;
    CollisionRequest *pair = msg.add_collision();
    pair->set_collision1(collision1);
    pair->set_collision2(collision2);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void changeSurface(gazebo::transport::PublisherPtr pub,
    const std::string & model,
    const std::string & link,
    const std::string & collision)
{
    ContactRequest msg;
    SurfaceRequest *req = msg.add_surface();
    req->set_model(model);
    req->set_link(link);
    req->set_collision(collision);
    gazebo::msgs::Surface *surface = new gazebo::msgs::Surface();
    gazebo::msgs::Friction *friction = new gazebo::msgs::Friction();

    friction->set_mu(10000.0);
    friction->set_mu2(10000.0);
    surface->set_allocated_friction(friction);
    req->set_allocated_surface(surface);

    pub->Publish(msg);
}

