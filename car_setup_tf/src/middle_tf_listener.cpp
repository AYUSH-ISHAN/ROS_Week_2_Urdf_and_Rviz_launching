#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>


void transformPoint(const tf::TransformListener& listener){
   //we'll create a point in the Untitled.008 frame that we'd like to transform to the base_link frame
   geometry_msgs::PointStamped mid_wheel_point;
   mid_wheel_point.header.frame_id = "Untitled.008";
   
  //we'll just use the most recent transform available for our simple example
   mid_wheel_point.header.stamp = ros::Time();
  
     //just an arbitrary point in space
   mid_wheel_point.point.x = 1.0;
   mid_wheel_point.point.y = 1.0;
   mid_wheel_point.point.z = 0.0;
  
   try{
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", mid_wheel_point, base_point);
        printf("\n");
        ROS_INFO("Untitled.008: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
            mid_wheel_point.point.x, mid_wheel_point.point.y, mid_wheel_point.point.z,
            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        printf("Eular angles with repect to the Untitled_1 coordinate system = x: 1.047, y: -1.047, z: -1.578  in radians    \n");
   }
   catch(tf::TransformException& ex){
       ROS_ERROR("Received an exception trying to transform a point from \"Untitled.008\" to \"base_link\": %s", ex.what());
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