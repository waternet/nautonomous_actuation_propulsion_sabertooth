#include "../include/nautonomous_propulsion_sabertooth/watchdog.hpp"

void run_watchdog(){
 
    status_msg.name = "actuation watchdog";
    ros::Rate rate(10);

	while (ros::ok()) {	

        check_status();
     
		rate.sleep();
		ros::spinOnce();
	}
}

void check_status(){

    std::string responce;
    ros::Rate rate(10);
   
    if(actuation_serial->isOpen()){
        responce = "";    
        actuation_serial->read(responce, 1);

        if(!responce.empty()){
            //ROS_INFO("Actuation alive");
            temp_status_msg.level = 0; // OK status
            temp_status_msg.message = "actuation running";    
        }else{
            //Read again after 100 ms
            rate.sleep();
            actuation_serial->read(responce, 1);
        
            if(!responce.empty()){
                //ROS_INFO("Actuation alive");
                temp_status_msg.level = 0; // OK status
                temp_status_msg.message = "actuation running";
            }else{
                //ROS_INFO("Actuation not alive");
                temp_status_msg.level = 1; // WARN status
                temp_status_msg.message = "actuation not running";
            }
        }
        actuation_serial->flushInput();

    }else{
        temp_status_msg.level = 2; // ERROR status
        temp_status_msg.message = "Serial not connected/open";    
    }

    //Publish only new status
    if(temp_status_msg.level != status_msg.level){
        status_msg.level = temp_status_msg.level;
        status_msg.message = temp_status_msg.message;

        watchdog_publisher.publish(status_msg);       
    }    
}
