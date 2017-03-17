# nautonomous actuation synchronizer {#nautonomous_actuation_synchronizer}

Package used to write commands to the serial output. command_actuation_node.cpp.cpp receives them from the nautonomous_actuation_selector, sabertooth.cpp transforms them to sabertooth commands, send_actuation.cpp send them to the serial. serial.cpp has the actual serial functions. 


command_actuation_node.cpp
Main file, subscribes to multiplexed topics.

sabertooth.cpp Transforms the Twist message to an actual sabertooth command.

send_actuation.cpp Initilizes serial, sending commands.

serial.cpp Serial functions.


## Files
[Include](../../doxygen_nautonomous/html/dir_0e1bf686876a66f778c9a6403953ffcc.html)  |  [Src](../../doxygen_nautonomous/html/dir_24f9e5a3f98ba5b1652caba176a493c4.html)
