#include "doodbot/doodbot.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "wxyz");
    ros::Time::init();
    Doodbot junkdood(argc, argv);

    // junkdood.draw_board();
    // junkdood.draw_X(0, 0);
    // junkdood.draw_O(0, 2);
    // junkdood.draw_X(1, 1);
    // junkdood.draw_X(2, 0);
    // junkdood.draw_O(2, 2);

    // junkdood.letsplay();

    junkdood.playgamepad();

    return 0;
}