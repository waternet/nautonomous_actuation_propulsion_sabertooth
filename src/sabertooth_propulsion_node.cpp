#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_propulsion_node.h>

/**
 * \brief Shutdown handler, deinit serial, call ros::shutdown()
 * \param int sig
 * \return null 
 */
void shutdownHandler(int sig)
{
  // Deinit serial before shutdown down.
  delete sabertooth_propulsion;
  sabertooth_propulsion = nullptr;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

/**
 * Main for command actuation node, subscribes to topics, call init serial
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuation_propulsion_sabertooth_node");

    sabertooth_propulsion = new SabertoothPropulsion();

    // When the program exists, go to the shutdown handler first.
    signal(SIGINT, shutdownHandler);

    ros::Rate rate(10);

    while(ros::ok)
    {
        sabertooth_propulsion->checkWatchdog();
        rate.sleep();
        ros::spinOnce();
    }
    
    delete sabertooth_propulsion;
    sabertooth_propulsion = nullptr;

    return 0;
}
