#include "ros/ros.h"
#include <ros/package.h>
#include "polynomial_traj.h"
#include <Eigen/Eigen>
#include <fstream>


#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "polynomial_traj/PvayCommand.h"


nav_msgs::Path path;

geometry_msgs::PoseStamped pose;

PolynomialTraj gl_traj;


ros::Publisher paths_pub;

Eigen::MatrixXd pos;
Eigen::Vector3d start_vel, end_vel, start_acc, end_acc;
Eigen::VectorXd time_time;


std::ofstream csvFile;


int main(int argc, char* argv[])
{
    // Initialise the node
    ros::init(argc, argv, "polynomial_traj_test");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");
    // Display the namespace of the node handle
    ROS_INFO_STREAM("PLAIN CPP NODE] namespace of nh = " << nh.getNamespace());

    // 创建 CSV 文件
    std::string filename = "trajectory.csv";
    csvFile.open(filename.c_str());
    // 写入 CSV 文件头
    csvFile << "p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z" << std::endl;
    paths_pub = nh.advertise<nav_msgs::Path>("real_path", 100, true);


    // 设定有几个点
    pos.resize(3,6);

    pos <<  0,13,13,10,9,0,
            0,2,-8,-8, -2,0,
            0,1,1,1,   1,0;
    ROS_INFO_STREAM(pos);
    std::cout << pos(0,1) << std::endl;
    std::cout << "Key point set" << std::endl;
    for(int i = 0; i < 6; i++)
    {
        pose.header.frame_id = "world";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = pos(0,i);
        pose.pose.position.y = pos(1,i);
        pose.pose.position.z = pos(2,i);
        path.poses.push_back(pose);
    }

    start_vel << 0.5,0.5,0;
    end_vel <<  0.5,0.5,0;
    start_acc<< 0,0,0;
    end_acc <<  0,0,0;

    ROS_INFO("OK");
    time_time.resize(5);
    time_time << 10,5,3,8,15;

    ROS_INFO("OK");

    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time_time);
    ROS_INFO("OK");

    gl_traj.init();
    double global_duration_ = gl_traj.getTimeSum();

    ROS_INFO("global_duration = %f s", global_duration_);
    double ts = 0.1;
    int index = 0;

    for (double t = 0; t < global_duration_; t += ts)
    {   
        if(ros::ok() == false)
        {
            break;
        }

        
        Eigen::Vector3d pt = gl_traj.evaluate(t);

        Eigen::Vector3d pt_v = gl_traj.evaluateVel(t);
        Eigen::Vector3d pt_a = gl_traj.evaluateAcc(t);


        csvFile << pt(0) << "," << pt(1) << "," << pt(2) << ","
        << pt_v(0) << "," << pt_v(1) << "," << pt_v(2) << ","
        << pt_a(0) << "," << pt_a(1) << "," << pt_a(2) << std::endl;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped cur_pos;
        cur_pos.header.frame_id = "world";
        cur_pos.header.stamp = ros::Time::now();
        cur_pos.header.seq = index;
        cur_pos.pose.position.x = pt(0);
        cur_pos.pose.position.y = pt(1);
        cur_pos.pose.position.z = pt(2);
        index++;
        path.poses.push_back(cur_pos);
        paths_pub.publish(path);
        // pub cmd quadrotor_msgs::PositionCommand
        ros::Duration(ts).sleep();
        ros::spinOnce();
    }
    ros::spin();
    csvFile.close();

    // Main has ended, return 0
    return 0;
}