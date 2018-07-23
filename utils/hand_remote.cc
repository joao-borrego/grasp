/*!
    \file utils/hand_remote.cc
    \brief TODO

    TODO

    \author João Borrego : jsbruglie
*/

#include "hand_remote.hh"

int main(int _argc, char **_argv)
{
    // Command-line arguments
    std::string config_file, robot;
    parseArgs(_argc, _argv, config_file, robot);

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Interface
    Interface io;
    // Init interface with config file
    io.init(config_file, robot);
    // Main loop
    io.loop();

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

//////////////////////////////////////////////////
const std::string getUsage(const char* argv_0)
{
    return \
        "usage:   " + std::string(argv_0) + " [options]\n" +
        "options: -c <robot configuration yaml>\n"  +
        "         -r <robot>\n";
}

//////////////////////////////////////////////////
void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot)
{
    int opt;
    bool c, r;

    while ((opt = getopt(argc,argv,"c: r:")) != EOF)
    {
        switch (opt)
        {
            case 'c':
                c = true; cfg_dir = optarg;    break;
            case 'r':
                r = true; robot = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!c || !r) {
        std::cerr << getUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

    debugPrint("Parameters:\n" <<
        "   Robot configuration yaml '" << cfg_dir << "'\n" <<
        "   Robot                    '" << robot << "'\n");
}

