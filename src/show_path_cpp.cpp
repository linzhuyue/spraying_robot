#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
using namespace std;

class ShowPathUR
{
  public:
    float tf_trans_x;
    float tf_trans_y;
    float tf_trans_z;
    float tf_quater_x;
    float tf_quater_y;
    float tf_quater_z;
    float tf_quater_w;

    void tf_pose_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
};
void ShowPathUR::tf_pose_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
//    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",msg->transforms[0].transform.translation.x,msg->transforms[0].transform.translation.y,msg->transforms[0].transform.translation.z);
//    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",msg->transforms[0].transform.rotation.x,msg->transforms[0].transform.rotation.y,msg->transforms[0].transform.rotation.z,msg->transforms[0].transform.rotation.w);
//    ROS_INFO("child_frame_id-> tool0_controller: [%s]",msg->transforms[0].child_frame_id.c_str());
    string A (msg->transforms[0].child_frame_id.c_str());
    string B="tool0_controller";
    int m=A.compare (B);
//    if(msg->transforms[0].child_frame_id.c_str()=="tool0_controller")
    if(m==0)
    {
        tf_trans_x=msg->transforms[0].transform.translation.x;
         ROS_INFO("Position-> x: [%f]",tf_trans_x);
        tf_trans_y=msg->transforms[0].transform.translation.y;
        tf_trans_z=msg->transforms[0].transform.translation.z;
        tf_quater_x=msg->transforms[0].transform.rotation.x;
        tf_quater_y=msg->transforms[0].transform.rotation.y;
        tf_quater_z=msg->transforms[0].transform.rotation.z;
        tf_quater_w=msg->transforms[0].transform.rotation.w;
    }
    ROS_INFO("msg->transforms[0].child_frame_id.c_str()---[%s]",msg->transforms[0].child_frame_id.c_str());
    /*tf_trans_x
    tf_trans_y
    tf_trans_z
    tf_quater_x
    tf_quater_y
    tf_quater_z
    tf_quater_w*/
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);
    ShowPathUR showpathur;
    ros::Subscriber path_sub = ph.subscribe<tf2_msgs::TFMessage>("tf",10, &ShowPathUR::tf_pose_callback,&showpathur);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="base";

    int cnt=1;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        current_time = ros::Time::now();
//        tf_trans_x += cnt;

        geometry_msgs::PoseStamped this_pose_stamped;

        this_pose_stamped.pose.position.x = showpathur.tf_trans_x;
        this_pose_stamped.pose.position.y = showpathur.tf_trans_y;
        this_pose_stamped.pose.position.z = showpathur.tf_trans_z;

        //geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
        this_pose_stamped.pose.orientation.x = showpathur.tf_quater_x;
        this_pose_stamped.pose.orientation.y = showpathur.tf_quater_y;
        this_pose_stamped.pose.orientation.z =showpathur.tf_quater_z;
        this_pose_stamped.pose.orientation.w = showpathur.tf_quater_w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="base";
        path.poses.push_back(this_pose_stamped);
        ROS_INFO("Orientation in main-> x [%f]",this_pose_stamped.pose.position.x);
//        cout<<this_pose_stamped.pose.position.x
        path_pub.publish(path);
        ros::spinOnce();               // check for incoming messages
//        ros::spin();
        last_time = current_time;
        loop_rate.sleep();
//        cnt+=0.1;
    }

    return 0;
}
