#include <ros/ros.h>
#include <std_msgs/Bool.h>

using namespace std;

void collisionCallback(const std_msgs::Bool::ConstPtr& msg)
{
if(msg->data == true){
ROS_INFO("Collision ahead");
}
}

int main(int argc, char **argv){

ros::init(argc, argv, "listener_node");
ros::NodeHandle nh;
ros::Subscriber collision_sub = nh.subscribe("Collision", 1, collisionCallback);

ros::spin();

return 0;
}
