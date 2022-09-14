#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Header.h>

// pointcloud calculate
#include <Eigen/Eigen>
#include <condition_variable>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// std
#include <memory>
#include <vector>
#include <list>
#include <mutex>
#include <thread>
#include <condition_variable>

typedef pcl::PointXYZI PointT;
typedef typename pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;

struct Pose
{
    double time;
    std_msgs::Header header;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f pose;
    Eigen::Affine3f T;

    Pose(): time(0){}
};

std::list<PointCloudPtr> cloud_buffer;
std::list<Pose> pose_buffer;
std::list<double> cloudtime_buffer;
std::mutex mtx_cloud;
std::mutex mtx_pose;

// camera to body
float rad_c_b_x;
float rad_c_b_y;
float rad_c_b_z;

// laser to body
float rad_l_b_x;
float rad_l_b_y;
float rad_l_b_z;

float time_one_frame;

// ros pub handler
ros::Publisher pub_cloud;
ros::Publisher pub_odom_camera;
ros::Publisher pub_odom_body;

void PointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr raw_msg)
{
    PointCloudPtr cloud_in(new PointCloud);
    cloud_in->reserve(raw_msg->width);
    pcl::fromROSMsg(*raw_msg, *cloud_in);

    {
        std::lock_guard<std::mutex> lock(mtx_cloud);
        cloud_buffer.push_back(cloud_in);
        cloudtime_buffer.push_back(raw_msg->header.stamp.toSec());
    }
}

void CameraHandler(const nav_msgs::Odometry::ConstPtr raw_msg)
{
    Pose p;
    p.time = raw_msg->header.stamp.toSec();
    p.header = raw_msg->header;
    p.header.frame_id = "world";
    p.orientation.w() = raw_msg->pose.pose.orientation.w;
    p.orientation.x() = raw_msg->pose.pose.orientation.x;
    p.orientation.y() = raw_msg->pose.pose.orientation.y;
    p.orientation.z() = raw_msg->pose.pose.orientation.z;
    p.pose.x() = raw_msg->pose.pose.position.x;
    p.pose.y() = raw_msg->pose.pose.position.y;
    p.pose.z() = raw_msg->pose.pose.position.z;
    p.T.linear() = p.orientation.toRotationMatrix();
    p.T.translation() = p.pose;

    {
        std::lock_guard<std::mutex> lock(mtx_pose);
        pose_buffer.push_back(p);
    }

}

void Process()
{
    Eigen::Affine3f T_c_b, T_b_c;
    Eigen::Affine3f T_l_b, T_b_l;
    // camera to body
    T_c_b.linear() = Eigen::AngleAxisf(rad_c_b_z, Eigen::Vector3f::UnitZ())*
                     Eigen::AngleAxisf(rad_c_b_y, Eigen::Vector3f::UnitY())*
                     Eigen::AngleAxisf(rad_c_b_x, Eigen::Vector3f::UnitX())
                     .toRotationMatrix();
    T_b_c = T_c_b.inverse();

    // laser to body
    T_l_b.linear() = Eigen::AngleAxisf(rad_l_b_z, Eigen::Vector3f::UnitZ())*
                     Eigen::AngleAxisf(rad_l_b_y, Eigen::Vector3f::UnitY())*
                     Eigen::AngleAxisf(rad_l_b_x, Eigen::Vector3f::UnitX())
                     .toRotationMatrix();
    T_b_l = T_l_b.inverse();

    // laser to camera
    auto T_l_c = T_c_b * T_l_b;
    auto T_c_l = T_l_b * T_c_b;

    // body0 is world
    // T_cn_w = T_cn_b0 = T_c0_b0 * T_cn_c0
    //                  = T_c_b * T_cn_c0
    // T_ln_w = T_c_b * T_cn_c0 * T_l_c

    // useful utils
    auto has_new_data = [&]()->bool
        {return !cloud_buffer.empty() && !pose_buffer.empty();};
    auto to_ros_msg = [&](const Eigen::Affine3f& T, nav_msgs::Odometry& msg)
        {
            auto& ori = msg.pose.pose.orientation;
            auto& pos = msg.pose.pose.position;
            Eigen::Quaternionf q(T.linear());
            auto& t = T.translation();
            ori.w = q.w();
            ori.x = q.x();
            ori.y = q.y();
            ori.z = q.z();
            pos.x = t.x();
            pos.y = t.y();
            pos.z = t.z();
        };
    ros::Rate r(40);
    
    while(ros::ok())
    {
        r.sleep();

        Eigen::Vector3f point_vec;
        Pose pose_before, pose_after;
        PointCloudPtr cloud;
        double time_cloud;

        // load data
        {
            std::lock_guard<std::mutex> lock1(mtx_cloud);
            std::lock_guard<std::mutex> lock2(mtx_pose);
            if(has_new_data())
            {
                cloud = cloud_buffer.front();
                time_cloud = cloudtime_buffer.front();

                // find two pose
                for(auto iter = pose_buffer.begin(); iter != pose_buffer.end(); iter++)
                {
                    if(iter->time > time_cloud)
                    {
                        if(iter != pose_buffer.begin())
                        {
                            iter--;
                            pose_before = *iter;
                            iter++;

                            for(; iter != pose_buffer.end(); iter++)
                            {
                                if(iter->time > time_cloud + time_one_frame)
                                {
                                    pose_after = *iter;
                                    break;
                                }
                            }
                        }

                        break;
                    }
                }

                if(pose_before.time != 0 && pose_after.time != 0)
                {
                    cloud_buffer.pop_front();
                    cloudtime_buffer.pop_front();
                    while(!pose_buffer.empty() && pose_buffer.front().time <= pose_before.time)
                    {
                        pose_buffer.pop_front();
                    }
                }
                else
                {
                    if(pose_before.time != 0)
                    {
                        ROS_WARN_STREAM_NAMED("Processor", "Not get enough pose msg");
                        ROS_INFO_STREAM_NAMED("Processor", "cloud time: " << ros::Time(time_cloud) << " ~ " << ros::Time(time_cloud + time_one_frame));
                    }
                    else
                    {
                        ROS_WARN_STREAM_NAMED("Processor", "Disguard early point cloud");
                        ROS_INFO_STREAM_NAMED("Processor", "cloud time: " << ros::Time(time_cloud) << " ~ " << ros::Time(time_cloud + time_one_frame));
                        cloud_buffer.pop_front();
                        cloudtime_buffer.pop_front();
                    }

                    ROS_INFO_STREAM_NAMED("Processor", "first msg time: " << ros::Time(pose_buffer.front().time));
                    ROS_INFO_STREAM_NAMED("Processor", "last msg time: " << ros::Time(pose_buffer.back().time));
                    continue;
                }
            }
            else
            {
                continue;
            }
        }

        // transform cloud
        for (size_t i = 0; i < cloud->size(); i++)
        {
            auto& point = cloud->at(i);
            point_vec.x() = point.x;
            point_vec.y() = point.y;
            point_vec.z() = point.z;

            float dt = point.intensity - int(point.intensity);
            dt = (dt + time_cloud - pose_before.time) / (pose_after.time - pose_before.time);
            
            auto r_cn_c0 = pose_before.orientation.slerp(dt, pose_after.orientation);
            auto t_cn_c0 = pose_before.pose + (pose_after.pose - pose_before.pose) * dt;
            r_cn_c0.normalize();

            point_vec = T_c_b * (r_cn_c0 * T_l_c * point_vec + t_cn_c0);

            point.x = point_vec.x(); 
            point.y = point_vec.y();
            point.z = point_vec.z();
        }

        // publish cloud
        {
            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*(cloud), msg_cloud);
            msg_cloud.header.stamp.fromSec(time_cloud);
            msg_cloud.header.frame_id = "world";
            pub_cloud.publish(msg_cloud);

            // publish odom
            auto& T_cn_c0 = pose_before.T;

            // actually calculating T_cn_w
            nav_msgs::Odometry msg_camera;
            auto T_cn_w = T_c_b * T_cn_c0;
            to_ros_msg(T_cn_w, msg_camera);
            msg_camera.header = pose_before.header;
            pub_odom_camera.publish(msg_camera);

            // actually calculating T_ln_w
            // T_ln_w = T_c_b * T_cn_c0 * T_l_c
            nav_msgs::Odometry msg_laser;
            auto T_ln_w = T_c_b * T_cn_c0 * T_l_c;
            to_ros_msg(T_ln_w, msg_laser);
            msg_laser.header = pose_before.header;
            pub_odom_body.publish(msg_laser);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_processor");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public("");

    nh_private.param("time_one_frame", time_one_frame, 1.0f);

    nh_private.param("transform_camera_to_body_x", rad_c_b_x, 0.0f);
    nh_private.param("transform_camera_to_body_y", rad_c_b_y, 0.0f);
    nh_private.param("transform_camera_to_body_z", rad_c_b_z, 0.0f);
    rad_c_b_x = rad_c_b_x / 180.0f * M_PI;
    rad_c_b_y = rad_c_b_y / 180.0f * M_PI;
    rad_c_b_z = rad_c_b_z / 180.0f * M_PI;

    nh_private.param("transform_laser_to_body_x", rad_l_b_x, 0.0f);
    nh_private.param("transform_laser_to_body_y", rad_l_b_y, 0.0f);
    nh_private.param("transform_laser_to_body_z", rad_l_b_z, 0.0f);
    rad_l_b_x = rad_l_b_x / 180.0f * M_PI;
    rad_l_b_y = rad_l_b_y / 180.0f * M_PI;
    rad_l_b_z = rad_l_b_z / 180.0f * M_PI;

    auto thread_process = std::thread(Process);

    auto sub_cloud = nh_public.subscribe<sensor_msgs::PointCloud2>("local_cloud", 20, PointCloudHandler);
    auto sub_imu = nh_public.subscribe<nav_msgs::Odometry>("/odom", 50, CameraHandler);
    pub_cloud = nh_public.advertise<sensor_msgs::PointCloud2>("world_cloud", 2);
    pub_odom_camera = nh_public.advertise<nav_msgs::Odometry>("odom_camera", 2);
    pub_odom_body = nh_public.advertise<nav_msgs::Odometry>("odom_laser", 2);

    ros::spin();

    thread_process.join();

    return 0;
}