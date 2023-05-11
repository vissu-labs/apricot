#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class PathMarker
{
private:
    ros::NodeHandle nh;
    size_t id{0};
    size_t MAX_SIZE{200};
    visualization_msgs::Marker pathMarker;
    visualization_msgs::MarkerArray pathMarkerArray;
    ros::Subscriber goalSub = nh.subscribe("/move_base/goal", 1, &PathMarker::goalSubCallback, this);
    ros::Subscriber cmdSub = nh.subscribe("/cmd_vel", 1, &PathMarker::cmdVelCb, this);
    ros::Subscriber odomSub = nh.subscribe("/odom", 1, &PathMarker::odomCb, this);
    ros::Publisher markerPub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    void odomCb(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // std::cout << "In odom cb" <<std::endl;
        pathMarker.pose.position.x = odom->pose.pose.position.x;
        pathMarker.pose.position.y = odom->pose.pose.position.y;
        pathMarker.pose.orientation = odom->pose.pose.orientation;
    }

    void goalSubCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &goal)
    {
        std::cout << "In goal cb" << std::endl;
        pathMarkerArray.markers.clear();
        id = 0;
        pathMarker.action = visualization_msgs::Marker::DELETEALL;
        publishMarker();
        pathMarker.action = visualization_msgs::Marker::ADD;
    };

    void cmdVelCb(const geometry_msgs::Twist::ConstPtr &vel)
    {
        // std::cout << "In cmd cb" <<std::endl;

        if (abs(vel->linear.x) > 0.01)
        {

            publishMarker();
        }
    }

public:
    PathMarker(ros::NodeHandle &nh, visualization_msgs::Marker &pathMarker)
    {
        this->pathMarker = pathMarker;
        this->nh = nh;
    };

    void publishMarker()
    {
        // std::cout << "In publish" <<std::endl;
        pathMarker.id = id;
        id += 1;
        if (id > MAX_SIZE)
        {
            pathMarkerArray.markers.erase(pathMarkerArray.markers.begin(), pathMarkerArray.markers.begin() + 1);
        }
        pathMarker.header.stamp = ros::Time::now();
        pathMarkerArray.markers.push_back(pathMarker);
        markerPub.publish(pathMarkerArray);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_marker");
    ros::NodeHandle nh;
    // ros::Rate r(10);
    visualization_msgs::Marker pathMarker;

    pathMarker.action = visualization_msgs::Marker::ADD;
    pathMarker.type = visualization_msgs::Marker::CUBE;
    pathMarker.header.frame_id = "map";

    pathMarker.ns = "path";
    pathMarker.id = 0;
    pathMarker.scale.x = 0.81;
    pathMarker.scale.y = 0.51;
    pathMarker.scale.z = 0.01;
    pathMarker.color.a = 1.0;
    pathMarker.color.r = 0.0f;
    pathMarker.color.g = 1.0f;
    pathMarker.color.b = 0.0f;
    pathMarker.pose.position.z = 0.0f;
    PathMarker marker{nh, pathMarker};

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}