#include "iwp_claw_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iwp_claw_node");

    ros::NodeHandle nh;

    ros::NodeHandle private_nh("~");

    IWP_CLAW_NODE iwp_claw_node(nh, private_nh);

    ros::Rate loop_rate(100);

    while (ros::ok()) {

        iwp_claw_node.update();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
