#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <turtlesim/SetPen.h>
#include <sstream>
#include<vector>

using namespace std;

struct Edges{
  double x1;
  double y1;

  double x2;
  double y2;
};

ros::Publisher turtle_controller;
ros::Subscriber turtle_pose_subscriber;

turtlesim::Pose turtlesim_pose;


 double calculateDistance(Edges edge){
   return sqrt((edge.x2-edge.x1)*(edge.x2-edge.x1) + (edge.y2-edge.y1)*(edge.y2-edge.y1));
 }

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;

  //ROS_INFO("Current pose is x = %f  y = %f  theta = %f", pose_message->x, pose_message->y, pose_message->theta);
}

void move(double speed, double distance){

  geometry_msgs::Twist vel_msg;


   vel_msg.linear.x =0.01 + abs(speed);

   vel_msg.linear.y =0;
   vel_msg.linear.z =0;

   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;


   double current_distance = 0.0;
   Edges edge{turtlesim_pose.x, turtlesim_pose.y,turtlesim_pose.x, turtlesim_pose.y};
   ros::Rate loop_rate(100);
   do{
     //Proportional speed
     vel_msg.linear.x = 0.01 + abs(speed)*(1-current_distance/distance);
	   turtle_controller.publish(vel_msg);
	   //update edge based on current position
     edge.x2 = turtlesim_pose.x;
     edge.y2 = turtlesim_pose.y;
	   current_distance = calculateDistance(edge);
	   ros::spinOnce();
	   loop_rate.sleep();
	   //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
   }while(current_distance<distance); //run until expected distance is traversed
   vel_msg.linear.x =0;
   turtle_controller.publish(vel_msg);

}

void rotate (double angular_speed, double relative_angle){

	geometry_msgs::Twist vel_msg;

	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;

	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;

     //Set angular velocity based on direction of rotation
	   if (relative_angle > 0)
	   		   vel_msg.angular.z =abs(angular_speed) + 0.01;
	   	   else
	   		   vel_msg.angular.z = -abs(angular_speed) - 0.01;


	   double current_angle = turtlesim_pose.theta;
	   ros::Rate loop_rate(1000);
	   do{
		   turtle_controller.publish(vel_msg);
       //calculate angular velocity proportionally
       vel_msg.angular.z = ((relative_angle>0)? 0.01:-0.01) + abs(angular_speed)*(1- abs((turtlesim_pose.theta-current_angle)<-3.14159  ? turtlesim_pose.theta-current_angle + 2*3.14159 : turtlesim_pose.theta-current_angle)/abs(relative_angle));

		   ros::spinOnce();
		   loop_rate.sleep();

	   }while(abs((turtlesim_pose.theta-current_angle)<-3.14159  ? turtlesim_pose.theta-current_angle + 2*3.14159 : turtlesim_pose.theta-current_angle   )<abs(relative_angle));

     vel_msg.angular.z =0;
	   turtle_controller.publish(vel_msg);
}



int main(int argc, char **argv)
{

  //Create vector of edges to traverse
  std::vector<Edges> path(16);

  path.push_back({0.0,0.0,2.0,0.0});
  path.push_back({2.0,0.0,2.0,2.0});
  path.push_back({2.0,2.0,0.0,2.0});
  path.push_back({0.0,2.0,0.0,0.0});
  path.push_back({0.0,0.0,2.0,2.0});
  path.push_back({2.0,2.0,1.0,3.0});
  path.push_back({1.0,3.0,0.0,2.0});
  path.push_back({0.0,2.0,2.0,0.0});

  path.push_back({2.2,-0.2 ,2.2,3.2});
  path.push_back({2.2,3.2 ,-0.2,3.2});
  path.push_back({-0.2,3.2 ,-0.2,-0.2});
  path.push_back({-0.2,-0.2 ,2.2,-0.2});


  path.push_back({2.2,-0.2 ,2.2,3.2});
  path.push_back({2.2,3.2 ,-0.2,3.2});
  path.push_back({-0.2,3.2 ,-0.2,-0.2});
  path.push_back({-0.2,-0.2 ,2.2,-0.2});



   ros::init(argc, argv, "field_planner");
   ros::NodeHandle n;

   double pi = 3.14159;
   ros::ServiceClient SetPenClient = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
   turtlesim::SetPen srv;
   srv.request.r = 255;
   srv.request.g = 255;
   srv.request.b = 255;
   srv.request.width = 0.1;
   srv.request.off = 0;

   if (SetPenClient.call(srv))
   {
     ROS_INFO("Setting pen parameters");
   }

   turtle_controller = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
   turtle_pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

   //Temp variables for calculating speed and angular velocity and some memory variables
   double relative_angle=0;
   double distance = 0;
   double angular_speed = 1;
   double speed = 5;
   double old_relative_angle = 0;
   double current_path_angle;
   double last_pose_x = 0.0;
   double last_pose_y = 0.0;

   //Loop over each edge and call move and rotate functions accordingly
   for(int i=0; i<path.size();i++){

     //identify jumps in edge list and switch pen on and off
     if(last_pose_x != path[i].x1 || last_pose_y - path[i].y1){
       current_path_angle = atan2(path[i].y1 - last_pose_y,path[i].x1-last_pose_x);
       Edges temp_edge{last_pose_x, last_pose_y, path[i].x1, path[i].y1};
        distance = calculateDistance(temp_edge);
       srv.request.off = 1;

       if (SetPenClient.call(srv))
       {
         ROS_INFO("Setting pen off");
       }
       last_pose_x = path[i].x1;
       last_pose_y = path[i].y1;

       i--;
   }
   else{
     current_path_angle = atan2(path[i].y2-path[i].y1,path[i].x2-path[i].x1);

      distance = calculateDistance(path[i]);
      srv.request.off = 0;

      if (SetPenClient.call(srv))
      {
        ROS_INFO("Setting pen on");
      }

              last_pose_x = path[i].x2;
              last_pose_y = path[i].y2;
             // last_pose_x = turtlesim_pose.x;
             // last_pose_y = turtlesim_pose.y;

   }

    //Rotate if there is a change in angles
     relative_angle = current_path_angle - old_relative_angle;
     old_relative_angle = current_path_angle;
     relative_angle = relative_angle<(-pi)? relative_angle + 2*pi : relative_angle;
     ROS_INFO("relative_angle = %f", relative_angle);
       rotate(angular_speed, relative_angle);
    //Move to cover the edge distance
     move(speed,distance);
     ROS_INFO("Current pose is x = %f  y = %f  theta = %f", turtlesim_pose.x, turtlesim_pose.y, turtlesim_pose.theta);
     //old_relative_angle = relative_angle;


   }
   ROS_INFO("Done Printing!");
   //ros::spin();
   if ( ros::ok() )
{
    ros::shutdown();
}


 }
