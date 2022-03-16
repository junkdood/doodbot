#include "doodbot/doodbot.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "wxyz");
    ros::Time::init();
    doodbot jundood(argc, argv);

    jundood.draw_X_test();

    return 0;
}