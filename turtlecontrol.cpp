#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

#define KP_linear 0.8
#define KI_linear 0.001
#define KP_angular 0.7
#define KI_angular 0.001
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double PI=3.14;


double degrees2radians(double angle_in_degrees);
void finalOrientation (double relativeAngle,double esumo);
void poseCallBack (const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal (turtlesim::Pose goal_pose,double esumx,double esumt,double relative_angle_radians);
double getDistance(double x1, double y1, double x2, double y2);
turtlesim::Pose userInput ();


int main (int argc,char **argv){
    


    turtlesim::Pose goal_pose;

    ros::init(argc,argv,"turtlesimcontrol");
    ros::NodeHandle n;


    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);   //notify about publishing to the nodes
    ros::Rate loop_rate(100);

    std::cout<<"enter desired x coordinate: ";
    std::cin>>goal_pose.x;
    std::cout<<"enter desired y coordinate: ";
    std::cin>>goal_pose.y;
    std::cout<<"enter desired angle (degrees): ";
    std::cin>>goal_pose.theta;


    pose_subscriber=n.subscribe("/turtle1/pose",2500,poseCallBack);    //turtle loctaion
    double distance_tolerance=0.01;
    double angle_tolerance=0.01;
   
    double esumx=0;
    double esumo=0;
    double esumt=0;
    int i=0;
    while(ros::ok()){
        
        if(getDistance(goal_pose.x,goal_pose.y,turtlesim_pose.x,turtlesim_pose.y)>distance_tolerance){
        double alpha = atan2(goal_pose.y,goal_pose.x);   //angle between the x axis and the turtles desired direction
        double beta = atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x);
        double relativeangle = beta-turtlesim_pose.theta;
        esumx+=(getDistance(goal_pose.x,goal_pose.y,turtlesim_pose.x,turtlesim_pose.y))*0.01;
        
        esumt+=relativeangle*0.01;
        
        moveGoal(goal_pose,esumx, esumt,relativeangle);
        }
        
        else{
            double turtle_theta=turtlesim_pose.theta;
            while (abs(turtle_theta)>PI){
                if (turtle_theta<-(PI))
                turtle_theta+=2*PI;
                else
                turtle_theta-=2*PI;
            }
            if((degrees2radians(goal_pose.theta)-turtle_theta)>angle_tolerance){
                esumo+=(degrees2radians(goal_pose.theta)-turtle_theta)*0.01;
                finalOrientation(degrees2radians(goal_pose.theta)-turtle_theta,esumo);
                printf("relative angle: %.3f\nturtle theta: %.3f\n",degrees2radians(goal_pose.theta)-turtle_theta,turtle_theta);
            }
            else{
        
        
        esumx=0;
        esumo=0;
        esumt=0;
        goal_pose=userInput();
            }
    
        }
        printf("location - x: %.3f, y: %.3f, theta: %.3f\n",turtlesim_pose.x,turtlesim_pose.y,turtlesim_pose.theta);
        
        
        ros::spinOnce();
        loop_rate.sleep();
        

    }
    

    return 0;
}

 turtlesim::Pose userInput (){
    turtlesim::Pose goal_pose;
    std::cout<<"enter desired x coordinate: ";
    std::cin>>goal_pose.x;
    std::cout<<"enter desired y coordinate: ";
    std::cin>>goal_pose.y;
    std::cout<<"enter desired angle (degrees): ";
    std::cin>>goal_pose.theta;
    return goal_pose;
}


double degrees2radians(double angle_in_degrees){
return angle_in_degrees*PI/180.0;
}



void finalOrientation (double relativeAngle,double esumo){
geometry_msgs::Twist vel_msg;
vel_msg.linear.x=0.0;
vel_msg.linear.z=0.0;
vel_msg.linear.y=0.0;
vel_msg.angular.x=0.0;
vel_msg.angular.y=0.0;

vel_msg.angular.z=abs(KP_angular*relativeAngle+KI_angular*esumo);
velocity_publisher.publish(vel_msg);
}

void poseCallBack (const turtlesim::Pose::ConstPtr & pose_message){   //save the message data
    turtlesim_pose.x=pose_message->x;
    turtlesim_pose.y=pose_message->y;
    turtlesim_pose.theta=pose_message->theta;
}

void moveGoal (turtlesim::Pose goal_pose,double esumx,double esumt,double relative_angle_radians){     //publish the velocity message
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x=abs(KP_linear*(getDistance(goal_pose.x,goal_pose.y,turtlesim_pose.x,turtlesim_pose.y))+KI_linear*esumx);
    vel_msg.angular.z=abs(KP_angular*relative_angle_radians+KI_angular*esumt);

    vel_msg.linear.z=0.0;
    vel_msg.linear.y=0.0;
    vel_msg.angular.x=0.0;
    vel_msg.angular.y=0.0;
    printf("speed - linear x: %.3f, angular z: %.3f\n", vel_msg.linear.x, vel_msg.angular.z);
   
    
    velocity_publisher.publish(vel_msg);
}



double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

