#include <ros/ros.h>
#include <fstream>
#include <boost/bind.hpp>
#include "message_to_log.h"

void logMessage(MessageToLog msgToLog, std::string filePath) {
    msgToLog.saveTUMFormat(filePath);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg, std::string filePath) {
    MessageToLog msgToLog(msg);
    logMessage(msgToLog, filePath);
}

void poseCallbackPoseStamped(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string filePath) {
    MessageToLog msgToLog(msg);
    logMessage(msgToLog, filePath);
}

void poseCallbackPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                           std::string filePath) {
    MessageToLog msgToLog(msg);
    logMessage(msgToLog, filePath);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_listener");

    ros::NodeHandle n("~");
    ROS_INFO("Start pose_listener...");
    std::string topic;
    std::string topic_type;
    std::string filepath;
//    filepath = "trajectory.txt";
    n.getParam("topic", topic);
    n.getParam("type", topic_type);
    n.param<std::string>("output_file", filepath, "trajectory.txt");
    ROS_INFO("Topic: %s", topic.c_str());
    ROS_INFO("Output file: %s", filepath.c_str());
    std::ifstream infile(filepath);
    if (infile.good()) {
        std::cerr << "ERROR: Output File Exists: " << filepath << std::endl;
        return 1;
    }

    ros::Subscriber sub; // This variable must be declared here, otherwise the subscriber goes out of scope and unsubscribes.
    if (topic_type == "PS") {
        ROS_INFO("Topic type: geometry_msgs::PoseStamped");
        sub = n.subscribe<geometry_msgs::PoseStamped>(topic, 1000,
                                                      boost::bind(poseCallbackPoseStamped, _1, boost::ref(filepath)));
    } else if (topic_type == "PCS") {
        ROS_INFO("Topic type: geometry_msgs::PoseWithCovarianceStamped");
        sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic, 1000,
                                                                    boost::bind(poseCallbackPoseWithCovarianceStamped,
                                                                                _1, boost::ref(filepath)));
    } else if (topic_type == "O") {
        ROS_INFO("Topic type: nav_msgs::Odometry");
        sub = n.subscribe<nav_msgs::Odometry>(topic, 1000, boost::bind(poseCallback, _1, boost::ref(filepath)));
    } else {
        std::cerr << "ERROR: Unsupported topic type. _type parameter must be one of these: [O, PCS, PS]" << std::endl;
        return -1;
    }
    ros::spin();

    return 0;
}
