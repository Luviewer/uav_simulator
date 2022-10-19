#include "iwp_mavros.hpp"
#include "string"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iwp_mavros_node");

    ros::NodeHandle nh;

    ros::NodeHandle private_nh("~");

    ros::Rate loop_rate(100);

    ros::Publisher local_pub; // ros motor publisher

    local_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    local_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/global", 10);

    while (ros::ok()) {

        geometry_msgs::PoseStamped pos;

        pos.pose.position.x = atof(argv[1]);
        pos.pose.position.y = atof(argv[2]);
        pos.pose.position.z = atof(argv[3]);

        geometry_msgs::Quaternion q;

        q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, (-atof(argv[4]) + 90) / 57.3);

        pos.pose.orientation.x = q.x;
        pos.pose.orientation.y = q.y;
        pos.pose.orientation.z = q.z;
        pos.pose.orientation.w = q.w;

        local_pub.publish(pos);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
