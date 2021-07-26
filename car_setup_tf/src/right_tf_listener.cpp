#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>


void transformPoint(const tf::TransformListener& listener){
   //we'll create a point in the Untitled.001 frame that we'd like to transform to the base_link frame
   geometry_msgs::PointStamped r_wheel_point;
   r_wheel_point.header.frame_id = "Untitled.002";
   
  //we'll just use the most recent transform available for our simple example
   r_wheel_point.header.stamp = ros::Time();
  
     //just an arbitrary point in space
   r_wheel_point.point.x = 1.0;
   r_wheel_point.point.y = 1.0;
   r_wheel_point.point.z = 0.0;
  
   try{
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", r_wheel_point, base_point);
        printf("\n");
        ROS_INFO("Untitled.002: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
            r_wheel_point.point.x, r_wheel_point.point.y, r_wheel_point.point.z,
            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        printf("Eular angles with repect to the Untitled_1 coordinate system = x: -1.57, y: 0, z: 0  in radians  \n");
   }
   catch(tf::TransformException& ex){
       ROS_ERROR("Received an exception trying to transform a point from \"Untitled.002\" to \"base_link\": %s", ex.what());
   }
}
   
int main(int argc, char** argv){
     ros::init(argc, argv, "robot_tf_listener");
     ros::NodeHandle n;
   
     tf::TransformListener listener(ros::Duration(10));
  
     //we'll transform a point once every second
     ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
   
     ros::spin();
   
}