#pragma once

#include "iostream"
#include "string"
#include <sstream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "gazebo_msgs/ApplyBodyWrench.h"

#include "gazebo_msgs/ApplyJointEffort.h"

#include <math.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class SocketTCP {
private:
    int sockfd;

    const char* ip_addr = "127.0.0.1";

    const int port = 5762;

    struct sockaddr_in servaddr;

    const int maxsize = 4096;

public:
    void init()
    {
        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(ip_addr);
        servaddr.sin_port = htons(port);

        if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
            printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }
    }

    int getdata(char* buff) { return recv(sockfd, buff, maxsize, 0); }

    ~SocketTCP() { close(sockfd); }
};

class ParseData {
private:
    const uint8_t CLAW_CLOSE = 0xAE;
    const uint8_t CLAW_OPEN = 0xBE;
    const uint8_t CLAW_STOP = 0xCE;

    uint8_t buffer[50];

    int recv_claw_close;

    uint8_t _data_len;
    uint8_t _data_cnt;
    uint8_t state;

public:
    ParseData()
    {
        _data_len = 0;

        _data_cnt = 0;

        state = 0;

        recv_claw_close = 0;
    }

    void recv_all(uint8_t* data_buf, uint8_t num)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < (num - 1); i++)
            sum += *(data_buf + i);

        if (!(sum == *(data_buf + num - 1)))
            return; //??sum

        if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAA))
            return; //????

        if (*(data_buf + 4) == CLAW_CLOSE) {
            recv_claw_close = 1;
        } else if (*(data_buf + 4) == CLAW_STOP) {
            recv_claw_close = 0;
        } else if (*(data_buf + 4) == CLAW_OPEN) {
            recv_claw_close = -1;
        }
    }

    void recv_data(uint8_t data)
    {
        if (state == 0 && data == 0xAA) {
            state = 1;
            buffer[0] = data;
        } else if (state == 1 && data == 0xAA) {
            state = 2;
            buffer[1] = data;
        } else if (state == 2 && data < 0XF1) {
            state = 3;
            buffer[2] = data;
        } else if (state == 3 && data < 50) {
            state = 4;
            buffer[3] = data;
            _data_len = data;
            _data_cnt = 0;
        } else if (state == 4 && _data_len > 0) {
            _data_len--;
            buffer[4 + _data_cnt++] = data;
            if (_data_len == 0)
                state = 5;
        } else if (state == 5) {
            state = 0;
            buffer[4 + _data_cnt] = data;
            recv_all(buffer, _data_cnt + 5);
        } else
            state = 0;
    }

    int getState()
    {
        printf("recv_claw_close=%d\r\n", recv_claw_close);

        return recv_claw_close;
    }
};

class IWP_CLAW_NODE {
private:
    ros::NodeHandle nh;

    ros::NodeHandle private_nh;

    ros::ServiceClient client_joint;

    gazebo_msgs::ApplyJointEffort applyjoint;

    SocketTCP socket;

    ParseData parsedata;

public:
    IWP_CLAW_NODE();

    IWP_CLAW_NODE(ros::NodeHandle _nh, ros::NodeHandle _private_nh);

    ~IWP_CLAW_NODE() { socket.~SocketTCP(); }

    void open_claw();

    void close_claw();

    void update();
};

IWP_CLAW_NODE::IWP_CLAW_NODE(ros::NodeHandle _nh, ros::NodeHandle _private_nh)
    : nh(_nh)
    , private_nh(_private_nh)
{
    client_joint = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");

    ros::service::waitForService("/gazebo/apply_joint_effort");

    socket.init();
}

void IWP_CLAW_NODE::open_claw()
{
    applyjoint.request.joint_name = "usl_iwp::usl_iwp_claw_0::usl_iwp_claw_flange_joint";
    applyjoint.request.effort = 200;
    applyjoint.request.duration.nsec = 1e9;

    client_joint.call(applyjoint);

    applyjoint.request.joint_name = "usl_iwp::usl_iwp_claw_1::usl_iwp_claw_flange_joint";
    applyjoint.request.effort = 200;
    applyjoint.request.duration.nsec = 1e9;

    client_joint.call(applyjoint);
}

void IWP_CLAW_NODE::close_claw()
{
    applyjoint.request.joint_name = "usl_iwp::usl_iwp_claw_0::usl_iwp_claw_flange_joint";
    applyjoint.request.effort = -200;
    applyjoint.request.duration.nsec = 1e9;

    client_joint.call(applyjoint);

    applyjoint.request.joint_name = "usl_iwp::usl_iwp_claw_1::usl_iwp_claw_flange_joint";
    applyjoint.request.effort = -200;
    applyjoint.request.duration.nsec = 1e9;

    client_joint.call(applyjoint);
}

void IWP_CLAW_NODE::update()
{
    char buff[20];

    int len = socket.getdata(buff);

    if (len > 0) {
        for (int i = 0; i < len; i++) {
            parsedata.recv_data(buff[i]);
        }
    }
            
        if (parsedata.getState() == 1) {
            close_claw();
        } else if (parsedata.getState() == -1) {
            open_claw();
        }
}
