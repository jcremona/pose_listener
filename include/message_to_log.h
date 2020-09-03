//
// Created by jcremona on 2/9/20.
//

#ifndef POSE_LISTENER_MESSAGE_TO_LOG_H
#define POSE_LISTENER_MESSAGE_TO_LOG_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class MessageToLog {

public:
    // I decide to write multiple constructors
    // (instead of having only one constructor that receives geometry_msgs::Pose)
    // because additional information of &msg may be needed in the future.
    MessageToLog(const nav_msgs::Odometry::ConstPtr &msg);

    MessageToLog(const geometry_msgs::PoseStamped::ConstPtr &msg);

    MessageToLog(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void saveTUMFormat(std::string filePath);

private:
    float m_xPos;
    float m_yPos;
    float m_zPos;
    float m_qw;
    float m_qx;
    float m_qy;
    float m_qz;
    float timestamp;

    void PoseToLog(const geometry_msgs::Pose pose);
};


#endif //POSE_LISTENER_MESSAGE_TO_LOG_H
