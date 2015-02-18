#include "ros/ros.h"

#include "dynamixel_msgs/JointState.h" //Motor Joint State
#include "std_msgs/Float64.h" //Float motor cmd


//State callback
void stateCb(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    ROS_INFO("Current pos: %f",msg->current_pos); //See the JointState.msg for all the avaiable fields
}



int main(int argc, char* argv[])
{

        //initialize the ros node (mandatory)
    ros::init(argc,argv,"test_motors");
    ros::NodeHandle n; //you need this

        //create a publisher for the "left arm shoulder roll"
    ros::Publisher larm_s_r_pub = n.advertise<std_msgs::Float64> ("larm_s_r_controller/command",0);

        //create a subscriber for the "left arm shoulder roll"
    ros::Subscriber larm_s_r_sub=n.subscribe("larm_s_r_controller/state",0,stateCb); //we have to give a callback


        //50Hz
    ros::Rate loop_rate(50);

    while(ros::ok())
    {

        std_msgs::Float64 msg;
        msg.data=0.0;
        ROS_INFO("Sending: %f",msg.data); //ros log (quite usefull)
        larm_s_r_pub.publish(msg); //send the command

        ros::spinOnce();//Update the loop

        loop_rate.sleep();


    }

    return 0;

}
