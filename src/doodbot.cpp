#include "doodbot/doodbot.h"

doodbot::doodbot(int argc, char **argv){
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application mode (portName)");
        throw "usage wrong";
    }

    if(strcmp("sim", argv[1]) == 0){
        dobot = new Simulator_Interface();
    }
    else if(strcmp("real", argv[1]) == 0){
        if (argc < 3) {
            ROS_ERROR("[USAGE]Application real portName");
            throw "usage wrong";
        }
        dobot = new Hardware_Interface(argv[2]);
    }
    else{
        ROS_ERROR("[ERROR]mode not right");
        throw "mode not right";
    }
    
}

doodbot::~doodbot(){
    delete dobot;
}