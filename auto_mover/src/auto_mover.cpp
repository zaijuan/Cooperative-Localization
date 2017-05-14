#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "ar_sys/Trans_Repro_Info.h"
#include <geometry_msgs/PoseStamped.h>
#include "auto_mover/robot_state.h"
#include <fstream>
#include <ostream>
#include <vector>


class AutoMoveRobot
{
public:
    AutoMoveRobot();

    struct Point_2d{
        double X;
        double Y;
    };

private:
    ros::NodeHandle nh_;

    bool START;
    bool is_Apollo_starting_position_stored;
    bool is_Boreas_starting_position_stored;
    bool MOVE_APOLLO_BACK_TO_START;
    bool MOVE_BOREAS_BACK_TO_START;

    std::string current_robot_state;
    std::string who_is_leader;
    bool moving_state_published;

    bool To_move_Apollo_back;
    bool To_move_Boreas_back;
    bool x_reached;
    bool y_reached;
    bool z_reached;

    int stored_transform_num;
    std::vector<tf::Transform> transform_stream;

    Point_2d current_move_goal;
    int current_move_goal_ID;

    int store_data_to_file_num;
    std::vector<double> record_localization_error;
    std::vector<Point_2d> record_groudtruth;
    std::vector<Point_2d> record_localization;

    double bigGoal[20][2] = { { 0.0, 0.0 },
                              { 0.0, 0.42 }, { 0.0, 0.42 },
                              { 0.0, 0.42 }, { 0.0, 0.42 },
                              { 0.0, 0.42 }, { 0.15, 0.0 },
                              { 0.15, 0.0 }, { 0.15, 0.0 },
                              { 0.15, 0.0 }, { 0.15, 0.0 },//before this moves Boreas,after this moves Apollo.
                              { 0.0, -0.4 }, { -0.15, 0.0 },
                              { 0.0, -0.4 }, { -0.15, 0.0 },
                              { 0.0, -0.4 }, { -0.15, 0.0 },
                              { 0.0, -0.4 }, { -0.15, 0.0 },
                              { 0.0, 0.4 }
                            }; //increacement or decrecement of each moving target.

    geometry_msgs::Twist twist_boreas;
    geometry_msgs::Twist twist_apollo;

    tf::Transform T_B_camleft_B_backboard;
    tf::Transform T_A_camfront_B_backboard;

    tf::Transform T_world_A_camfront, T_world_A_camfront_static;
    tf::Transform T_world_B_camleft, T_world_B_camleft_static;

    tf::Transform T_A_camfront_old_B_backboard, T_A_camfront_new_B_backboard;
    tf::Transform T_A_camfront_B_backboard_old, T_A_camfront_B_backboard_new;

    tf::Transform T_A_camfront_A_base;
    //this describes the transform from apollo coordinate frame to apollo's camfront frame.
    tf::Transform T_A_camfront_Target_B_backboard;
    tf::Transform T_B_backboard_A_base_Target;
    tf::Transform T_B_backboard_A_base_Temp;
    tf::Transform T_A_base_Target_A_base_Temp;

    tf::Transform T_B_backboard_B_base;
    //this describes the transform from boreas coordinate frame to boreas's camleft frame.
    tf::Transform T_A_camfront_B_backboard_Target;
    tf::Transform T_A_camfront_B_base_Target;
    tf::Transform T_A_camfront_B_base_Temp;
    tf::Transform T_B_base_Target_B_base_Temp;

    tf::Transform T_Apollo_starting_position;
    tf::Transform T_Boreas_starting_position;

    auto_mover::robot_state robot_state_srv;
    ros::ServiceClient robot_state_client_;

    ros::Publisher twist_Boreas_pub, twist_Apollo_pub;
    ros::Publisher B_camleft_transform_pub, A_camfront_transform_pub;
    ros::Publisher localization_error_pub;

    ros::Subscriber transRepro_sub_, groundtruth_BoreasCamleft_sub_, groundtruth_ApolloCamfront_sub_;

    void arsysTransReproCallback(const ar_sys::Trans_Repro_Info::ConstPtr& msg);
    void BoreasCamleftGroundtruthCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
    void ApolloCamFrontGroundtruthCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
};

//Initialize the Class--AutoMoveRobot
AutoMoveRobot::AutoMoveRobot():
    START(false),
    is_Apollo_starting_position_stored(false),is_Boreas_starting_position_stored(false),
    MOVE_APOLLO_BACK_TO_START(false),MOVE_BOREAS_BACK_TO_START(false),
    current_robot_state("Apollo_follows_dist_near"),//This setting is for getting accurate starting point.
    who_is_leader("none"),To_move_Apollo_back(false), To_move_Boreas_back(false),
    x_reached(false), y_reached(false), z_reached(false), moving_state_published(false),
    stored_transform_num(0), current_move_goal_ID(1), store_data_to_file_num(0){

    //Transform without noise;
    //T_B_camleft_B_backboard = tf::Transform(tf::Quaternion(1, 0, 0, 0),tf::Vector3(0.1, 0, -0.38));

    //*****Transform with noise.
    T_B_camleft_B_backboard = tf::Transform(tf::Quaternion(0.999599,-3.63795e-05,0.0267007,0.00940703),tf::Vector3(0.101534,-0.00252455,-0.398743));//(1.15cm,3mm,1.8cm)

    //this describes the transform from Apollo coordinate frame to Apollo's camfront frame.
    T_A_camfront_A_base = tf::Transform(tf::Quaternion(0.5,-0.5,0.5,0.5),tf::Vector3(0, 0.15, -0.2));
    T_A_camfront_Target_B_backboard = tf::Transform(tf::Quaternion( 1, 0, 0, 0), tf::Vector3(0, 0, 0.539));
    T_B_backboard_A_base_Target = T_A_camfront_Target_B_backboard.inverse() * T_A_camfront_A_base;

    //this describes the transform from boreas coordinate frame to boreas's camleft frame.
    T_B_backboard_B_base = tf::Transform(tf::Quaternion(-0.5,0.5,0.5,0.5),tf::Vector3(0,-0.15,-0.18));
    T_A_camfront_B_backboard_Target = tf::Transform(tf::Quaternion( 1, 0, 0, 0), tf::Vector3(0, 0, 0.539));
    T_A_camfront_B_base_Target = T_A_camfront_B_backboard_Target * T_B_backboard_B_base;

    current_move_goal.X = 0;
    current_move_goal.Y = 0;

    twist_Boreas_pub = nh_.advertise<geometry_msgs::Twist>("/Boreas/Twist", 1);
    twist_Apollo_pub = nh_.advertise<geometry_msgs::Twist>("/Apollo/Twist", 1);

    B_camleft_transform_pub = nh_.advertise<geometry_msgs::TransformStamped>("/T_world_Boreas_camleft", 10);
    A_camfront_transform_pub = nh_.advertise<geometry_msgs::TransformStamped>("/T_world_Apollo_camfront", 10);
    localization_error_pub = nh_.advertise<std_msgs::Float64>("/relative_localization_error", 10);

    transRepro_sub_ = nh_.subscribe<ar_sys::Trans_Repro_Info>("/ar_sys/Trans_Repro_Info", 10, &AutoMoveRobot::arsysTransReproCallback, this);
    groundtruth_BoreasCamleft_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/BoreasCamleft/true_pose", 10, &AutoMoveRobot::BoreasCamleftGroundtruthCallback, this);
    groundtruth_ApolloCamfront_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ApolloCamfront/true_pose", 10, &AutoMoveRobot::ApolloCamFrontGroundtruthCallback, this);

    robot_state_client_ = nh_.serviceClient<auto_mover::robot_state>("/AutoRobotStateSrv");
}



void AutoMoveRobot::BoreasCamleftGroundtruthCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    if( !is_Boreas_starting_position_stored ){

        T_Boreas_starting_position = tf::Transform(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
                                                   tf::Vector3(-(msg->pose.position.y-1.1002), msg->pose.position.x+1.1, msg->pose.position.z));
        is_Boreas_starting_position_stored = true;

        ros::Duration(5).sleep(); //Make sure sptam_coop has been well initialized before Boreas moves.
    }

    tf::Vector3 BoreasCamLeft_groundtruth_position=tf::Vector3( -(msg->pose.position.y-1.1002), msg->pose.position.x+1.1, msg->pose.position.z-0.03945 );
    tf::Vector3 estimated_position=tf::Vector3(T_world_B_camleft.getOrigin().getX(), T_world_B_camleft.getOrigin().getZ(), -T_world_B_camleft.getOrigin().getY());

    double error_x = BoreasCamLeft_groundtruth_position.getX() - estimated_position.getX();
    double error_y = BoreasCamLeft_groundtruth_position.getY() - estimated_position.getY();

    std_msgs::Float64 error_relative;
    error_relative.data = std::sqrt( error_x*error_x + error_y*error_y );
    localization_error_pub.publish(error_relative);

    store_data_to_file_num++;

    if( ( (store_data_to_file_num%2) ==0 ) && current_robot_state=="Boreas_is_moving" && !MOVE_APOLLO_BACK_TO_START && !MOVE_BOREAS_BACK_TO_START ){

        Point_2d current_ground_truth_;
        current_ground_truth_.X = BoreasCamLeft_groundtruth_position.getX();
        current_ground_truth_.Y = BoreasCamLeft_groundtruth_position.getY();

        Point_2d current_localization_;
        current_localization_.X = estimated_position.getX();
        current_localization_.Y = estimated_position.getY();

        record_localization_error.push_back(error_relative.data);
        record_groudtruth.push_back(current_ground_truth_);
        record_localization.push_back(current_localization_);
    }

    if( record_localization_error.size()==50 ){

        std::ofstream outfile1, outfile2, outfile3;

        outfile1.open("/home/zaijuan/Sim_Scene/results/caterpillar_localization_error.txt", std::fstream::app|std::fstream::out);
        outfile2.open("/home/zaijuan/Sim_Scene/results/caterpillar_ground_truth.txt",std::fstream::app|std::fstream::out);
        outfile3.open("/home/zaijuan/Sim_Scene/results/caterpillar_localization_results.txt", std::fstream::app|std::fstream::out);

        for (size_t t=0; t<record_localization_error.size(); t++){
            outfile1<<record_localization_error[t]<<std::endl;
            outfile2<<record_groudtruth[t].X<<" "<<record_groudtruth[t].Y<<std::endl;
            outfile3<<record_localization[t].X<<" "<<record_localization[t].Y<<std::endl;
        }

        outfile1.close();
        outfile2.close();
        outfile3.close();

        record_localization_error.clear();
        record_groudtruth.clear();
        record_localization.clear();

        store_data_to_file_num = 0;
    }


    if( ( current_move_goal_ID<=10 || current_move_goal_ID==19 ) && who_is_leader=="Boreas" ){//this moves Boreas to predefined position

        if( !moving_state_published ){

            moving_state_published = true;

            current_robot_state = "Boreas_is_moving";
            robot_state_srv.request.robot_state.data = current_robot_state;

            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            if (robot_state_passed)
                ROS_INFO_STREAM("The state: Boreas_is_moving has been successfully passed to sptam_coop.");
            if (!robot_state_passed)
                ROS_INFO_STREAM("The state: Boreas_is_moving has NOOOT been successfully passed to sptam_coop.");

            current_move_goal.X = BoreasCamLeft_groundtruth_position.getX() + bigGoal[current_move_goal_ID][0];
            current_move_goal.Y = BoreasCamLeft_groundtruth_position.getY() + bigGoal[current_move_goal_ID][1];
        }

        x_reached = false;
        y_reached = false;

        double x_difference = current_move_goal.X - BoreasCamLeft_groundtruth_position.getX();
        double y_difference = current_move_goal.Y - BoreasCamLeft_groundtruth_position.getY();

        if( fabs(x_difference)>0.02 )
            twist_boreas.linear.y = -1 * x_difference;
        else{
            x_reached = true;
            twist_boreas.linear.y = 0;
        }

        if( fabs(y_difference)>0.02 )
            twist_boreas.linear.x = 1 * y_difference;
        else{
            y_reached = true;
            twist_boreas.linear.x = 0;
        }

        twist_boreas.angular.z = 0;

        twist_Boreas_pub.publish(twist_boreas);
        ROS_INFO_STREAM("The real-time velocity of Boreas is:" << twist_boreas.linear.x << " " << twist_boreas.linear.y <<" "<<twist_boreas.angular.z);

        if( x_reached && y_reached ){

            moving_state_published = false;

            who_is_leader = "none";

            current_robot_state = "Boreas_stops_dist_far";
            robot_state_srv.request.robot_state.data = current_robot_state;

            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            if (robot_state_passed)
                ROS_INFO_STREAM("The state: Boreas_stops_dist_far has been successfully passed to sptam_coop.");
            if (!robot_state_passed)
                ROS_INFO_STREAM("The state: Boreas_stops_dist_far has NOOOT been successfully passed to sptam_coop.");

            ros::Duration(1.5).sleep();//Make sure sptam_coop node has got stable transform.

            current_move_goal_ID++;
        }
    }

    if( MOVE_BOREAS_BACK_TO_START ){

        current_move_goal.X = T_Boreas_starting_position.getOrigin().getX();
        current_move_goal.Y = T_Boreas_starting_position.getOrigin().getY();

        x_reached = false;
        y_reached = false;

        double x_difference = current_move_goal.X - BoreasCamLeft_groundtruth_position.getX();
        double y_difference = current_move_goal.Y - BoreasCamLeft_groundtruth_position.getY();

        if( fabs(x_difference)>0.02 )
            twist_boreas.linear.y = -1 * x_difference;
        else{
            x_reached = true;
            twist_boreas.linear.y = 0;
        }

        if( fabs(y_difference)>0.02 )
            twist_boreas.linear.x = 1 * y_difference;
        else{
            y_reached = true;
            twist_boreas.linear.x = 0;
        }

        twist_boreas.angular.z = 0;

        twist_Boreas_pub.publish(twist_boreas);
        ROS_INFO_STREAM("The real-time velocity of the Boreas is:" << twist_boreas.linear.x << " " << twist_boreas.linear.y <<" "<<twist_boreas.angular.z);

        if( x_reached && y_reached )
            ROS_INFO_STREAM("Boreas has moved to the starting point.");

    }

}



void AutoMoveRobot::ApolloCamFrontGroundtruthCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){

    if( !is_Apollo_starting_position_stored ){
        T_Apollo_starting_position = tf::Transform(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w),
                                                   tf::Vector3(-(msg->pose.position.y-1.0001), msg->pose.position.x+2.0497, msg->pose.position.z));
        is_Apollo_starting_position_stored = true;
    }

    if( current_move_goal_ID>=11 && current_move_goal_ID<=18 && who_is_leader=="Apollo" ){//this moves Apollo and Boreas follows

        tf::Vector3 ApolloCam_current_position=tf::Vector3(-(msg->pose.position.y-1.0001), msg->pose.position.x + 2.0497, msg->pose.position.z);

        if( !moving_state_published ){

            moving_state_published = true;

            current_robot_state = "Apollo_is_moving";
            robot_state_srv.request.robot_state.data = current_robot_state;

            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            if (robot_state_passed)
                ROS_INFO_STREAM("The state: Apollo_is_moving has been successfully passed to sptam_coop.");
            if (!robot_state_passed)
                ROS_INFO_STREAM("The state: Apollo_is_moving has NOOOT been successfully passed to sptam_coop.");

            current_move_goal.X = ApolloCam_current_position.getX() + bigGoal[current_move_goal_ID][0];
            current_move_goal.Y = ApolloCam_current_position.getY() + bigGoal[current_move_goal_ID][1];
        }

        x_reached = false;
        y_reached = false;

        double x_difference = current_move_goal.X - ApolloCam_current_position.getX();
        double y_difference = current_move_goal.Y - ApolloCam_current_position.getY();

        if( fabs(x_difference)>0.02 )
            twist_apollo.linear.y = -1 * x_difference;
        else{
            x_reached = true;
            twist_apollo.linear.y = 0;
        }

        if( fabs(y_difference)>0.02 )
            twist_apollo.linear.x = 1 * y_difference;
        else{
            y_reached = true;
            twist_apollo.linear.x = 0;
        }

        twist_apollo.angular.z = 0;

        twist_Apollo_pub.publish(twist_apollo);
        ROS_INFO_STREAM("The real-time velocity of Apollo is:" << twist_apollo.linear.x << " " << twist_apollo.linear.y <<" "<<twist_apollo.angular.z);

        if( x_reached && y_reached ){

            moving_state_published = false;
            who_is_leader = "none";

            current_robot_state = "Apollo_stops_dist_far";
            robot_state_srv.request.robot_state.data = current_robot_state;

            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            if (robot_state_passed)
                ROS_INFO_STREAM("The state: Apollo_stops_dist_far has been successfully passed to sptam_coop.");
            if (!robot_state_passed)
                ROS_INFO_STREAM("The state: Apollo_stops_dist_far has NOOOT been successfully passed to sptam_coop.");

            ros::Duration(1.5).sleep();//Make sure sptam_coop node has got stable transform.

            current_move_goal_ID++;
        }
    }

    if( MOVE_APOLLO_BACK_TO_START ){

        tf::Vector3 ApolloCam_current_position=tf::Vector3(-(msg->pose.position.y-1.0001), msg->pose.position.x + 1.9997, msg->pose.position.z);

        current_move_goal.X = T_Apollo_starting_position.getOrigin().getX();
        current_move_goal.Y = T_Apollo_starting_position.getOrigin().getY();

        x_reached = false;
        y_reached = false;

        double x_difference = current_move_goal.X - ApolloCam_current_position.getX();
        double y_difference = current_move_goal.Y - ApolloCam_current_position.getY();

        if( fabs(x_difference)>0.02 )
            twist_apollo.linear.y = -1 * x_difference;
        else{
            x_reached = true;
            twist_apollo.linear.y = 0;
        }

        if( fabs(y_difference)>0.02 )
            twist_apollo.linear.x = 1 * y_difference;
        else{
            y_reached = true;
            twist_apollo.linear.x = 0;
        }

        twist_apollo.angular.z = 0;

        twist_Apollo_pub.publish(twist_apollo);
        ROS_INFO_STREAM("The real-time velocity of Apollo is:" << twist_apollo.linear.x << " " << twist_apollo.linear.y <<" "<<twist_apollo.angular.z);

        if( x_reached && y_reached )
            ROS_INFO_STREAM("Apollo has moved to the starting point.");

    }

}


void AutoMoveRobot::arsysTransReproCallback(const ar_sys::Trans_Repro_Info::ConstPtr& msg)
{

    double x_coord,y_coord,z_coord;
    double x_rot, y_rot, z_rot, w_rot;

    x_coord = msg->transform_.transform.translation.x;
    y_coord = msg->transform_.transform.translation.y;
    z_coord = msg->transform_.transform.translation.z;
    x_rot = msg->transform_.transform.rotation.x;
    y_rot = msg->transform_.transform.rotation.y;
    z_rot = msg->transform_.transform.rotation.z;
    w_rot = msg->transform_.transform.rotation.w;

    T_A_camfront_B_backboard.setOrigin(tf::Vector3(x_coord, y_coord, z_coord));
    T_A_camfront_B_backboard.setRotation(tf::Quaternion(x_rot, y_rot, z_rot, w_rot));


    if( To_move_Apollo_back ){

        T_B_backboard_A_base_Temp = T_A_camfront_B_backboard.inverse() * T_A_camfront_A_base;
        T_A_base_Target_A_base_Temp = T_B_backboard_A_base_Target.inverse() * T_B_backboard_A_base_Temp;

        x_reached = false;
        y_reached = false;
        z_reached = false;

        tf::Quaternion Quaternion_A_base_Target_A_base_Temp = T_A_base_Target_A_base_Temp.getRotation();
        tf::Matrix3x3 Rotation_A_base_Target_A_base_Temp(Quaternion_A_base_Target_A_base_Temp);
        double apollo_roll, apollo_pitch, apollo_yaw;
        Rotation_A_base_Target_A_base_Temp.getRPY(apollo_roll, apollo_pitch, apollo_yaw); //The yaw we get here is for rotation around Z axis. And like the translation, the rotation angle is the opposite of the yaw angles we got here.

        if( fabs( T_A_base_Target_A_base_Temp.getOrigin().getX() )>0.02 )
            twist_apollo.linear.x = -1 * T_A_base_Target_A_base_Temp.getOrigin().getX();//the parameter is set larger in simulation.
        else{
            twist_apollo.linear.x = 0;
            x_reached = true;
        }

        if( fabs( T_A_base_Target_A_base_Temp.getOrigin().getY() )>0.02 )
            twist_apollo.linear.y = -1 * T_A_base_Target_A_base_Temp.getOrigin().getY(); //the parameter is set larger in simulation.
        else{
            twist_apollo.linear.y = 0;
            y_reached= true;
        }

        if( fabs( apollo_yaw )>0.04 ) //less than 2 degrees.
            twist_apollo.angular.z = -0.4 * apollo_yaw;
        else{
            twist_apollo.angular.z = 0;
            z_reached = true;
        }

        twist_Apollo_pub.publish(twist_apollo);

        if(x_reached && y_reached && z_reached){

            To_move_Apollo_back = false;

            current_robot_state = "Apollo_follows_dist_near";
            robot_state_srv.request.robot_state.data = current_robot_state;

            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            if (robot_state_passed)
                ROS_INFO_STREAM("The state: Apollo_follows_dist_near has been successfully passed to sptam_coop.");
            if (!robot_state_passed)
                ROS_INFO_STREAM("The state: Apollo_follows_dist_near has NOOOOT been successfully passed to sptam_coop.");

            ros::Duration(1.5).sleep();//Make sure sptam_ node has got stable transform.
        }

    }


    if( To_move_Boreas_back ){

        T_A_camfront_B_base_Temp = T_A_camfront_B_backboard * T_B_backboard_B_base;
        T_B_base_Target_B_base_Temp = T_A_camfront_B_base_Target.inverse() * T_A_camfront_B_base_Temp;

        x_reached = false;
        y_reached = false;
        z_reached = false;

        tf::Quaternion Quaternion_B_base_Target_B_base_Temp = T_B_base_Target_B_base_Temp.getRotation();
        tf::Matrix3x3 Rotation_B_base_Target_B_base_Temp(Quaternion_B_base_Target_B_base_Temp);
        double boreas_roll, boreas_pitch, boreas_yaw;
        Rotation_B_base_Target_B_base_Temp.getRPY(boreas_roll, boreas_pitch, boreas_yaw); //The yaw we get here is for rotation around Z axis. And like the translation, the rotation angle is the opposite of the yaw angles we got here.

        if( fabs( T_B_base_Target_B_base_Temp.getOrigin().getX() )>0.02 )
            twist_boreas.linear.x = -1 * T_B_base_Target_B_base_Temp.getOrigin().getX();//the parameter is set larger in simulation.
        else{
            twist_boreas.linear.x = 0;
            x_reached = true;
        }

        if( fabs( T_B_base_Target_B_base_Temp.getOrigin().getY() )>0.02 )
            twist_boreas.linear.y = -1 * T_B_base_Target_B_base_Temp.getOrigin().getY(); //the parameter is set larger in simulation.
        else{
            twist_boreas.linear.y = 0;
            y_reached= true;
        }

        if( fabs( boreas_yaw )>0.04 )//less than 2 degrees.
            twist_boreas.angular.z = -0.4 * boreas_yaw;
        else{
            twist_boreas.angular.z = 0;
            z_reached = true;
        }

        twist_Boreas_pub.publish(twist_boreas);

        if( x_reached && y_reached && z_reached ){

            To_move_Boreas_back = false;

            current_robot_state = "Boreas_follows_dist_near";
            robot_state_srv.request.robot_state.data = current_robot_state;

            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            if (robot_state_passed)
                ROS_INFO_STREAM("The state: Boreas_follows_dist_near has been successfully passed to sptam_coop.");
            if (!robot_state_passed)
                ROS_INFO_STREAM("The state: Boreas_follows_dist_near has NOOOT been successfully passed to sptam_coop.");

            ros::Duration(1.5).sleep();//Make sure sptam_ node has got stable transform.
        }
    }


    if( current_robot_state=="Boreas_stops_dist_far" ){//Boreas is leader and apollo is far.

        if( stored_transform_num<10 ){

            transform_stream.push_back(T_A_camfront_B_backboard);
            stored_transform_num++;
        }

        if( stored_transform_num==10){

            double trans_x = 0;
            double trans_y = 0;
            double trans_z = 0;
            double roll_sum  = 0;
            double pitch_sum = 0;
            double yaw_sum = 0;
            double roll_single = 0;
            double pitch_single = 0;
            double yaw_single = 0;

            for(int i=0;i<transform_stream.size();i++){
                trans_x += transform_stream.at(i).getOrigin().getX();
                trans_y += transform_stream.at(i).getOrigin().getY();
                trans_z += transform_stream.at(i).getOrigin().getZ();
                transform_stream.at(i).getBasis().getRPY(roll_single,pitch_single,yaw_single);
                roll_sum += roll_single;
                pitch_sum += pitch_single;
                yaw_sum += yaw_single;
            }

            trans_x = trans_x/transform_stream.size();
            trans_y = trans_y/transform_stream.size();
            trans_z = trans_z/transform_stream.size();
            roll_single = roll_sum/transform_stream.size();
            pitch_single = pitch_sum/transform_stream.size();
            yaw_single = yaw_sum/transform_stream.size();

            tf::Matrix3x3 rotation_base;
            tf::Quaternion quaternion_base;
            rotation_base.setEulerYPR(yaw_single,pitch_single,roll_single);
            rotation_base.getRotation(quaternion_base);

            T_A_camfront_old_B_backboard = tf::Transform(quaternion_base,tf::Vector3(trans_x, trans_y, trans_z));

            T_world_B_camleft_static = T_world_A_camfront * T_A_camfront_old_B_backboard * T_B_camleft_B_backboard.inverse();
            T_world_B_camleft = T_world_B_camleft_static;

            stored_transform_num = 0;
            transform_stream.clear();

            To_move_Apollo_back = true;

            ros::Duration(1.5).sleep();

            current_robot_state = "Apollo_is_moving";

            //            robot_state_srv.request.robot_state.data = current_robot_state;

            //            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            //            if (robot_state_passed)
            //                ROS_INFO_STREAM("The state: Apollo_is_moving has been successfully passed to sptam_coop.");
            //            if (!robot_state_passed)
            //                ROS_INFO_STREAM("The state: Apollo_is_moving has NOOOT been successfully passed to sptam_coop.");
        }

        return;
    }


    if( current_robot_state=="Apollo_follows_dist_near" ){ //Boreas is leader and apollo is near.

        if( stored_transform_num<10 ){

            transform_stream.push_back(T_A_camfront_B_backboard);
            stored_transform_num++;
        }

        if( stored_transform_num == 10){

            if (!START){

                double trans_x = 0;
                double trans_y = 0;
                double trans_z = 0;
                double roll_sum  = 0;
                double pitch_sum = 0;
                double yaw_sum = 0;
                double roll_single = 0;
                double pitch_single = 0;
                double yaw_single = 0;

                for(int i=0;i<transform_stream.size();i++){
                    trans_x += transform_stream.at(i).getOrigin().getX();
                    trans_y += transform_stream.at(i).getOrigin().getY();
                    trans_z += transform_stream.at(i).getOrigin().getZ();
                    transform_stream.at(i).getBasis().getRPY(roll_single,pitch_single,yaw_single);
                    roll_sum += roll_single;
                    pitch_sum += pitch_single;
                    yaw_sum += yaw_single;
                }

                trans_x = trans_x/transform_stream.size();
                trans_y = trans_y/transform_stream.size();
                trans_z = trans_z/transform_stream.size();
                roll_single = roll_sum/transform_stream.size();
                pitch_single = pitch_sum/transform_stream.size();
                yaw_single = yaw_sum/transform_stream.size();

                tf::Matrix3x3 rotation_base;
                tf::Quaternion quaternion_base;
                rotation_base.setEulerYPR(yaw_single,pitch_single,roll_single);
                rotation_base.getRotation(quaternion_base);

                T_A_camfront_new_B_backboard = tf::Transform(quaternion_base,tf::Vector3(trans_x, trans_y, trans_z));

                T_world_B_camleft = tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
                T_world_A_camfront_static = T_world_B_camleft * T_B_camleft_B_backboard * T_A_camfront_new_B_backboard.inverse();
                T_world_A_camfront = T_world_A_camfront_static;

                stored_transform_num = 0;
                transform_stream.clear();

                START = true;
                who_is_leader = "Boreas";

                return;
            }

            if (START){

                double trans_x = 0;
                double trans_y = 0;
                double trans_z = 0;
                double roll_sum  = 0;
                double pitch_sum = 0;
                double yaw_sum = 0;
                double roll_single = 0;
                double pitch_single = 0;
                double yaw_single = 0;

                for(int i=0;i<transform_stream.size();i++){
                    trans_x += transform_stream.at(i).getOrigin().getX();
                    trans_y += transform_stream.at(i).getOrigin().getY();
                    trans_z += transform_stream.at(i).getOrigin().getZ();
                    transform_stream.at(i).getBasis().getRPY(roll_single,pitch_single,yaw_single);
                    roll_sum += roll_single;
                    pitch_sum += pitch_single;
                    yaw_sum += yaw_single;
                }

                trans_x = trans_x/transform_stream.size();
                trans_y = trans_y/transform_stream.size();
                trans_z = trans_z/transform_stream.size();
                roll_single = roll_sum/transform_stream.size();
                pitch_single = pitch_sum/transform_stream.size();
                yaw_single = yaw_sum/transform_stream.size();

                tf::Matrix3x3 rotation_base;
                tf::Quaternion quaternion_base;
                rotation_base.setEulerYPR(yaw_single,pitch_single,roll_single);
                rotation_base.getRotation(quaternion_base);

                T_A_camfront_new_B_backboard = tf::Transform(quaternion_base,tf::Vector3(trans_x, trans_y, trans_z));

                T_world_A_camfront_static = T_world_B_camleft * T_B_camleft_B_backboard * T_A_camfront_new_B_backboard.inverse();
                T_world_A_camfront = T_world_A_camfront_static;

                stored_transform_num = 0;
                transform_stream.clear();

                ros::Duration(1.5).sleep();

                if( current_move_goal_ID<=10 || current_move_goal_ID==19){
                who_is_leader = "Boreas";
                current_robot_state = "Boreas_is_moving";
                }

                if( current_move_goal_ID>=11 && current_move_goal_ID!=19){
                      who_is_leader = "Apollo";
                      current_robot_state = "Apollo_is_moving";
                }

                if ( current_move_goal_ID==20 ){
                    MOVE_APOLLO_BACK_TO_START = true;
                    MOVE_BOREAS_BACK_TO_START = true;
                }

            }
        }

        return;
    }


    if( current_robot_state=="Apollo_stops_dist_far" ){//Apollo is leader and boreas is far.

        if( stored_transform_num< 10 ){

            transform_stream.push_back(T_A_camfront_B_backboard);
            stored_transform_num++;
        }

        if( stored_transform_num==10){

            double trans_x = 0;
            double trans_y = 0;
            double trans_z = 0;
            double roll_sum  = 0;
            double pitch_sum = 0;
            double yaw_sum = 0;
            double roll_single = 0;
            double pitch_single = 0;
            double yaw_single = 0;

            for(int i=0;i<transform_stream.size();i++){
                trans_x += transform_stream.at(i).getOrigin().getX();
                trans_y += transform_stream.at(i).getOrigin().getY();
                trans_z += transform_stream.at(i).getOrigin().getZ();
                transform_stream.at(i).getBasis().getRPY(roll_single,pitch_single,yaw_single);
                roll_sum += roll_single;
                pitch_sum += pitch_single;
                yaw_sum += yaw_single;
            }

            trans_x = trans_x/transform_stream.size();
            trans_y = trans_y/transform_stream.size();
            trans_z = trans_z/transform_stream.size();
            roll_single = roll_sum/transform_stream.size();
            pitch_single = pitch_sum/transform_stream.size();
            yaw_single = yaw_sum/transform_stream.size();

            tf::Matrix3x3 rotation_base;
            tf::Quaternion quaternion_base;
            rotation_base.setEulerYPR(yaw_single,pitch_single,roll_single);
            rotation_base.getRotation(quaternion_base);

            T_A_camfront_B_backboard_old = tf::Transform(quaternion_base,tf::Vector3(trans_x, trans_y, trans_z));

            T_world_A_camfront_static = T_world_B_camleft * T_B_camleft_B_backboard * T_A_camfront_B_backboard_old.inverse();
            T_world_A_camfront = T_world_A_camfront_static;

            stored_transform_num = 0;
            transform_stream.clear();

            To_move_Boreas_back = true;

            ros::Duration(1.5).sleep();

            current_robot_state = "Boreas_is_moving";

            //            robot_state_srv.request.robot_state.data = current_robot_state;

            //            bool robot_state_passed = robot_state_client_.call(robot_state_srv);
            //            if (robot_state_passed)
            //                ROS_INFO_STREAM("The state: Boreas_is_moving has been successfully passed to sptam_coop.");
            //            if (!robot_state_passed)
            //                ROS_INFO_STREAM("The state: Boreas_is_moving has NOOOT been successfully passed to sptam_coop.");
        }

        return;
    }


    if( current_robot_state=="Boreas_follows_dist_near" ){

        if( stored_transform_num<10 ){

            transform_stream.push_back(T_A_camfront_B_backboard);
            stored_transform_num++;
        }

        if( stored_transform_num==10 ){

            double trans_x = 0;
            double trans_y = 0;
            double trans_z = 0;
            double roll_sum  = 0;
            double pitch_sum = 0;
            double yaw_sum = 0;
            double roll_single = 0;
            double pitch_single = 0;
            double yaw_single = 0;

            for(int i=0;i<transform_stream.size();i++){
                trans_x += transform_stream.at(i).getOrigin().getX();
                trans_y += transform_stream.at(i).getOrigin().getY();
                trans_z += transform_stream.at(i).getOrigin().getZ();
                transform_stream.at(i).getBasis().getRPY(roll_single,pitch_single,yaw_single);
                roll_sum += roll_single;
                pitch_sum += pitch_single;
                yaw_sum += yaw_single;
            }

            trans_x = trans_x/transform_stream.size();
            trans_y = trans_y/transform_stream.size();
            trans_z = trans_z/transform_stream.size();
            roll_single = roll_sum/transform_stream.size();
            pitch_single = pitch_sum/transform_stream.size();
            yaw_single = yaw_sum/transform_stream.size();

            tf::Matrix3x3 rotation_base;
            tf::Quaternion quaternion_base;
            rotation_base.setEulerYPR(yaw_single,pitch_single,roll_single);
            rotation_base.getRotation(quaternion_base);

            T_A_camfront_B_backboard_new = tf::Transform(quaternion_base, tf::Vector3(trans_x, trans_y, trans_z));

            T_world_B_camleft_static = T_world_A_camfront * T_A_camfront_B_backboard_new * T_B_camleft_B_backboard.inverse();
            T_world_B_camleft = T_world_B_camleft_static;

            stored_transform_num = 0;
            transform_stream.clear();

            ros::Duration(1.5).sleep();

            if( current_move_goal_ID<=10 || current_move_goal_ID==19){
            who_is_leader = "Boreas";
            current_robot_state = "Boreas_is_moving";
            }


            if( current_move_goal_ID>=11 && current_move_goal_ID!=19){
            who_is_leader = "Apollo";
            current_robot_state = "Apollo_is_moving";
            }


            if ( current_move_goal_ID==20 ){
                MOVE_APOLLO_BACK_TO_START = true;
                MOVE_BOREAS_BACK_TO_START = true;
            }

        }
        return;
    }



    if ( START && current_robot_state=="Boreas_is_moving" ){

        T_world_A_camfront = T_world_A_camfront_static; //When Boreas is moving, it means Apollo is not. So it's camfront stays the same.
        T_world_B_camleft = T_world_A_camfront * T_A_camfront_B_backboard * T_B_camleft_B_backboard.inverse();

        tf::StampedTransform stampedTransform_world_B(T_world_B_camleft, msg->transform_.header.stamp, "world", "B_camleft");
        tf::StampedTransform stampedTransform_world_A(T_world_A_camfront, msg->transform_.header.stamp, "world", "A_camfront");

        geometry_msgs::TransformStamped transformMsg_world_B;
        geometry_msgs::TransformStamped transformMsg_world_A;

        tf::transformStampedTFToMsg(stampedTransform_world_B, transformMsg_world_B);
        tf::transformStampedTFToMsg(stampedTransform_world_A, transformMsg_world_A);

        B_camleft_transform_pub.publish(transformMsg_world_B);
        A_camfront_transform_pub.publish(transformMsg_world_A);  //now publish the Boreas's leftcam and Apollo's camfront's representation in world reference.
    }


    if ( START && current_robot_state=="Apollo_is_moving" ){

        T_world_B_camleft = T_world_B_camleft_static;
        T_world_A_camfront = T_world_B_camleft * T_B_camleft_B_backboard * T_A_camfront_B_backboard.inverse();

        tf::StampedTransform stampedTransform_world_B(T_world_B_camleft, msg->transform_.header.stamp, "world", "B_camleft");
        tf::StampedTransform stampedTransform_world_A(T_world_A_camfront, msg->transform_.header.stamp, "world", "A_camfront");

        geometry_msgs::TransformStamped transformMsg_world_B;
        geometry_msgs::TransformStamped transformMsg_world_A;

        tf::transformStampedTFToMsg(stampedTransform_world_B, transformMsg_world_B);
        tf::transformStampedTFToMsg(stampedTransform_world_A, transformMsg_world_A);

        B_camleft_transform_pub.publish(transformMsg_world_B);
        A_camfront_transform_pub.publish(transformMsg_world_A);  //now publish the Boreas's leftcam and Apollo's camfront's representation in world reference.
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_mover");
    AutoMoveRobot auto_move_robot_;
    ros::spin();
}
