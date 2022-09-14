// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <motor/RMDS.h>

// pointcloud calculate
#include <Eigen/Eigen>
#include <condition_variable>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// std
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>

typedef pcl::PointXYZI PointT;
typedef typename pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;

struct LaserScanData
{
    double relative_time;
    ros::Time stamp;
    std::shared_ptr<std::vector<PointCloudPtr>> rings;
};

// ros handler
ros::Publisher pub_cloud;
ros::Time node_start_time;

// hardward config
int line_num_per_frame;
size_t points_num;
float laser_motor_speed;
float laser_motor_bias;

// multi-thread utils
int buffer_size;
bool laser_scan_frame_ready;
std::condition_variable laser_scan_cv;
std::mutex laser_scan_mutex;
std::queue<std::shared_ptr<LaserScanData>> ready_buffer;
std::queue<std::shared_ptr<LaserScanData>> empty_buffer;

void LaserScanHandler(const sensor_msgs::LaserScanConstPtr &raw_scan_msg)
{
    static Eigen::Matrix4f T_rotate_scanplane_to_init_scanplane;

    // do some init
    {
        static bool need_init = true;
        if (need_init)
        {
            points_num = raw_scan_msg->intensities.size();
            for (int i = 0; i < buffer_size; i++)
            {
                std::shared_ptr<LaserScanData> data_ptr(new LaserScanData);
                data_ptr->rings = std::make_shared<std::vector<PointCloudPtr>>(line_num_per_frame);

                for (int j = 0; j < line_num_per_frame; j++)
                {
                    data_ptr->rings->at(j) = PointCloudPtr(new PointCloud);
                    data_ptr->rings->at(j)->reserve(points_num);
                }

                empty_buffer.push(data_ptr);
            }

            T_rotate_scanplane_to_init_scanplane = Eigen::Matrix4f::Identity();
            node_start_time = raw_scan_msg->header.stamp;

            need_init = false;
        }
    }

    // extract points from LaserScan msg
    static std::shared_ptr<LaserScanData> data_ptr;
    {
        const static float time_per_frame = raw_scan_msg->scan_time;
        const static float time_inc = raw_scan_msg->time_increment;
        const static float time_blind = time_per_frame - raw_scan_msg->intensities.size() * time_inc;
        const static float rad_plane_inc = laser_motor_speed * time_inc;
        static int cnt = 0;              // count how many msgs have gotten
        static float rad_plane = 0.0f;   // scan plane rad
        static float rad_plane_bias = 0.0f;
        static float time = 0.0f;

        // do some init before each collection
        if (0 == cnt)
        {
            std::lock_guard<std::mutex> lock(laser_scan_mutex);
            time = 0.0f;
            rad_plane_bias = 2 * M_PI - laser_motor_bias;

            if (!empty_buffer.empty())
            {
                data_ptr = empty_buffer.front();
                data_ptr->stamp = raw_scan_msg->header.stamp;
                data_ptr->relative_time = (data_ptr->stamp-node_start_time).toSec();
                empty_buffer.pop();
                ROS_DEBUG_STREAM_NAMED("LidarDriver", "Start collect new data: " << data_ptr->relative_time);
            }
            else
            {
                ROS_WARN_STREAM_NAMED("LidarDriver", "no enough space for new data");
                return;
            }
        }

        // cal scan plane normal
        auto &ranges = raw_scan_msg->ranges;
        auto &intensities = raw_scan_msg->intensities;
        auto range_max = raw_scan_msg->range_max;
        auto ring = data_ptr->rings->at(cnt);
        // custome urg_node which put current degree in intensities[0]
        rad_plane = 2 * M_PI - intensities[0] - rad_plane_bias;

        ring->clear();
        PointT p;
        float rad_point = raw_scan_msg->angle_min;
        Eigen::Vector4f p_local, p_motor;
        p_local[2] = 0;
        p_local[3] = 1;
        for (size_t i = 0; i < points_num; i++)
        {
            if (ranges[i] > 1.0f && ranges[i] < range_max)
            {
                p_local[0] = ranges[i] * cos(rad_point);
                p_local[1] = ranges[i] * sin(rad_point);

                auto cur_rad_plane = rad_plane - i * rad_plane_inc;
                T_rotate_scanplane_to_init_scanplane(1, 1) = cos(cur_rad_plane);
                T_rotate_scanplane_to_init_scanplane(1, 2) = -sin(cur_rad_plane);
                T_rotate_scanplane_to_init_scanplane(2, 1) = -T_rotate_scanplane_to_init_scanplane(1, 2);
                T_rotate_scanplane_to_init_scanplane(2, 2) = T_rotate_scanplane_to_init_scanplane(1, 1);

                p_motor = T_rotate_scanplane_to_init_scanplane * p_local;
                p.x = p_motor[0];
                p.y = p_motor[1];
                p.z = p_motor[2];
                p.intensity = cnt + time;

                ring->push_back(p);
            }

            rad_point += raw_scan_msg->angle_increment;
            time += time_inc;
        }

        // increace cnt & plane degree
        cnt = (cnt + 1) % line_num_per_frame;
        time += time_blind;

        if (0 != cnt)
            return;
    }

    // if get enough points than process thess points
    {
        std::lock_guard<std::mutex> lock(laser_scan_mutex);
        ready_buffer.push(data_ptr);
        laser_scan_frame_ready = true;
        laser_scan_cv.notify_all();
        ROS_DEBUG_STREAM_NAMED("LidarDriver", "Collect finish data: " << data_ptr->relative_time);
    }
}

void LaserScanProcess()
{
    PointCloudPtr cloud_out(new PointCloud);
    ros::Time sweep_start;

    std::shared_ptr<std::vector<PointCloudPtr>> rings(new std::vector<PointCloudPtr>(line_num_per_frame));
    for (int i = 0; i < line_num_per_frame; i++)
    {
        rings->at(i) = PointCloudPtr(new PointCloud);
        rings->at(i)->reserve(points_num);
    }

    while (ros::ok())
    {
        // we cannot lock too much time, there are only 0.025s for us to process data
        // in this little time, all we can do is copy data, func must release mutex asap
        // as for extract feature & publish, we can process them in next 0.975s
        {
            std::unique_lock<std::mutex> lock(laser_scan_mutex);
            laser_scan_cv.wait(lock, []
                               { return laser_scan_frame_ready || !ros::ok(); });

            laser_scan_frame_ready = false;
        }

        while (!ready_buffer.empty() && ros::ok())
        {
            // swap data
            {
                std::lock_guard<std::mutex> lock(laser_scan_mutex);
                auto data_ptr = ready_buffer.front();
                sweep_start = data_ptr->stamp;
                rings.swap(data_ptr->rings);
                ready_buffer.pop();
                empty_buffer.push(data_ptr);
                ROS_DEBUG_STREAM_NAMED("LidarDriver", "Start publish data: " << data_ptr->relative_time);
            }

            // copy data
            for (size_t i = 0; i < rings->size(); i++)
            {
                *cloud_out += *(rings->at(i));
            }

            // publish data
            {
                static sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
                pcl::toROSMsg(*cloud_out, *msg);
                msg->header.frame_id = "local";
                msg->header.stamp = sweep_start;
                pub_cloud.publish(msg);
            }

            cloud_out->clear();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_driver");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public("");

    // load param
    {
        float laser_rate = 0.0f;
        nh_private.param("buffer_size", buffer_size, 3);
        nh_private.param("laser_rate", laser_rate, 40.0f);
        nh_private.param("laser_motor_bias", laser_motor_bias, 0.0f);
        nh_private.param("laser_motor_speed", laser_motor_speed, 180.0f);
        nh_private.param("line_num", line_num_per_frame, int(std::round(180.0f / laser_motor_speed * laser_rate)));
        laser_motor_bias = laser_motor_bias / 180.0f * M_PI;
        laser_motor_speed = laser_motor_speed / 180.0f * M_PI;    
    }
    
    // start motor
    std::shared_ptr<Motor::RMDS> motor;
    {
        std::string dev;
        nh_private.param("motor_id", dev, std::string("/dev/ttyUSB0"));
        motor = std::make_shared<Motor::RMDS>(dev, 1);
        if(motor->rotate(laser_motor_speed / M_PI * 180.0f))
            ROS_INFO_STREAM_NAMED("LidarDriver", "motor started...");
        else
        {
            ROS_FATAL_STREAM_NAMED("LidarDriver", "cannot start motor, driver exit");
            return 1;
        }
    }

    
    pub_cloud = nh_public.advertise<sensor_msgs::PointCloud2>("local_cloud", 2);
    auto sub_laser_scan = nh_public.subscribe<sensor_msgs::LaserScan>("/scan", 10, LaserScanHandler);
    auto thread_process = std::thread(&LaserScanProcess);

    ros::spin();

    motor->pause();
    laser_scan_cv.notify_all();
    thread_process.join();

    return 0;
}