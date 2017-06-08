# nautonomous actuation synchronizer {#nautonomous_actuation_synchronizer}

Package used to write commands to the serial output. command_actuation_node.cpp receives them from the nautonomous_actuation_selector, sabertooth.cpp transforms them to sabertooth commands, send_actuation.cpp send them to the serial. serial.cpp has the actual serial functions. 


command_actuation_node.cpp
Main file, subscribes to multiplexed topics.

sabertooth.cpp Transforms the Twist message to an actual sabertooth command.

send_actuation.cpp Initilizes serial, sending commands.

serial.cpp Serial functions.

## Nodes
command_actuation_node

##Topics

###Subscribe
/multiplexed_propulsion, /multiplexed_conveyor, /multiplexed_lighting

###Publish
-- publishes commands to serial port --

## Files
[Include](dir_24f9e5a3f98ba5b1652caba176a493c4.html)  |  [Src](dir_0e1bf686876a66f778c9a6403953ffcc.html)

##Overview
![send_actuation.launch](../images/launch_send_actuation.png)
<br />
![legenda](../images/legenda.png)

