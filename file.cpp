#include"ros/ros.h"
#include"turtlesim/Pose.h"
#include"geometry_msgs/Twist.h"

class turtleMover
{
	ros::NodeHandle nh;
	ros::Publisher velPublisher;
	ros::Subscriber poseSubscriber;
	ros::Subscriber goalSubscriber;
	turtlesim::Pose pose;
	turtlesim::Pose goal;
	void poseCallback(const turtlesim::Pose& newPose);
	void goalCallback(const turtlesim::Pose& newGoal);
	public:turtleMover();
};

turtleMover::turtleMover()
{
	//register publisher with message type geometry_msgs::Twist named cmd_vel with queue size 10
	velPublisher=nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	
	//register pose subscriber. Onreceiving a message turtleMover::poseCallback is automatically called
	poseSubscriber=nh.subscribe("pose",10,&turtleMover::poseCallback,this);
	goalSubscriber=nh.subscribe("goal",10,&turtleMover::goalCallback,this);//register goal subscriber
	
	goal.x=-1;goal.y=-1;
	goal.theta=0; //currentgoal (overwritten when a message at /goal is received)
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"turtleMover");//register node with all re-mappings given at command line
	turtleMover mover; //constructor registers all publishers and subscribers 
	
	//spin disallows the program to terminate unless ROS exits. 
	//also allows transfer of control to registered callback functions when a message isreceived
	ros::spin();
	return 0;
}

//automaticallycalled on receiving a new goal on /goal topic
void turtleMover::goalCallback(const turtlesim::Pose& newGoal)
{
	goal=newGoal;
	ROS_INFO("[%f %f]", goal.x,goal.y);
}
		
//automatically called on receiving a new pose on /pose topic

void turtleMover::poseCallback(const turtlesim::Pose& newPose)
{
	pose=newPose;
	ROS_INFO("(%f %f %f %f)", goal.x,goal.y, pose.x,pose.y);
	if(goal.x>0) //valid goal              
	{
		double kp1=1.0, kp2=1.0;
		double dis=sqrt(pow(goal.x-pose.x,2)+pow(goal.y-pose.y,2));
		double v=kp1*dis;
		double goalAngle=atan2(goal.y-pose.y,goal.x-pose.x);
		double angleError=goalAngle-pose.theta;
		angleError=atan2(sin(angleError),cos(angleError)); //limit to lie between -pi to pi
		double omega=kp2*angleError;
		geometry_msgs::Twist vel;
		vel.linear.x=v; //liner speed is in the robot’s X direction
		vel.linear.y=0;
		vel.linear.z=0;
		vel.angular.x=0;
		vel.angular.y=0;
		vel.angular.z=omega;//angular speed is in the robot’s Z direction
		velPublisher.publish(vel);
	}
}