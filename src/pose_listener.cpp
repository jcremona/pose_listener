#include <ros/ros.h>
#include <ros/network.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <fstream>
#include <boost/bind.hpp>

// Generate string with current date time
//inline std::string getCurrentDateTime( std::string s ){
//    time_t now = time(0);
//    struct tm  tstruct;
//    char  buf[80];
//    tstruct = *localtime(&now);
//    if(s=="now")
//        strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
//    else if(s=="date")
//        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
//    return std::string(buf);
//};

// Log string message
inline void Logger( std::string filePath, std::string logMsg){

    std::ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app );
//    ofs << now << '\t' << logMsg << '\n';
    ofs << logMsg << std::endl;
    ofs.close();
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg, std::string& filepath)
{
    float m_xPos = msg->pose.pose.position.x;
    float m_yPos = msg->pose.pose.position.y;
    float m_zPos = msg->pose.pose.position.z;
    float m_qw = msg->pose.pose.orientation.w;
    float m_qx = msg->pose.pose.orientation.x;
    float m_qy = msg->pose.pose.orientation.y;
    float m_qz = msg->pose.pose.orientation.z;
    ros::Time stamp = msg->header.stamp;
    ROS_INFO("Pose: (%f, %f, %f, %f, %f, %f, %f, %f)", stamp.toSec(), m_xPos, m_yPos, m_zPos, m_qw, m_qx, m_qy, m_qz);

    std::stringstream ss;
    ss << std::fixed << stamp.toSec() << " " << m_xPos << " " << m_yPos << " " << m_zPos << " " << m_qw << " "<< m_qx << " " << m_qy << " " << m_qz;

    std::string T_b_w_str = ss.str();
    Logger(filepath, T_b_w_str);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_listener");

  ros::NodeHandle n("~");
  ROS_INFO("Start pose_listener...");
  std::string topic;
  std::string filepath;
  n.getParam("topic", topic);
  n.getParam("output", filepath);
  ROS_INFO("Topic: %s", topic.c_str());
  ROS_INFO("Output path: %s", filepath.c_str());
  ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>(topic, 1000, boost::bind(poseCallback, _1, boost::ref(filepath)));

  ros::spin();

  return 0;
}
