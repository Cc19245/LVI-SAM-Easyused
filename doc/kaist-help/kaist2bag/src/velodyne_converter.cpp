//
// Created by tao on 7/25/21.
//

#include "velodyne_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <math.h>

namespace kaist2bag
{

    VelodyneConverter::VelodyneConverter(const std::string &dataset_dir, const std::string &save_dir,
                                         const std::string &left_topic, const std::string &right_topic)
        : Converter(dataset_dir, save_dir), left_topic_(left_topic), right_topic_(right_topic)
    {
        left_bag_name_ = FilterSlash(left_topic_) + ".bag";
        // right_bag_name_ = FilterSlash(right_topic_) + ".bag";
    }

    int VelodyneConverter::Convert()
    {
        CheckAndCreateSaveDir();

        boost::filesystem::path left_bag_file = boost::filesystem::path(save_dir_) / left_bag_name_;
        boost::filesystem::path right_bag_file = boost::filesystem::path(save_dir_) / right_bag_name_;

        const std::string left_stamp_file = dataset_dir_ + "/" + default_left_stamp_file;
        const std::string left_data_dir = dataset_dir_ + "/" + default_left_data_dir;
        // const std::string right_stamp_file = dataset_dir_ + "/" + default_right_stamp_file;
        // const std::string right_data_dir = dataset_dir_ + "/" + default_right_data_dir;

        ROS_INFO("saving %s", left_bag_file.c_str());
        Convert(left_stamp_file, left_data_dir, left_bag_file.string(), left_topic_, "left_velodyne");
        ROS_INFO("done saving %s", left_bag_file.c_str());
        // ROS_INFO("saving %s", right_bag_file.c_str());
        // Convert(right_stamp_file, right_data_dir, right_bag_file.string(), right_topic_, "right_velodyne");
        // ROS_INFO("done saving %s", right_bag_file.c_str());

        return 0;
    }

    void VelodyneConverter::RecoverVLP16Timestamp(const VPointCloud::Ptr input_cloud,
                                                  RTPointCloud::Ptr output_cloud)
    {
        // TODO It is not collected in order from top to bottom
        double VLP16_time_block_[1824][16];
        for (unsigned int w = 0; w < 1824; w++)
        {
            for (unsigned int h = 0; h < 16; h++)
            {
                VLP16_time_block_[w][h] =
                    h * 2.304 * 1e-6 + w * 55.296 * 1e-6; /// VLP_16 16*1824
            }
        }

        double lidar_fov_down = -15.0;
        double lidar_fov_resolution = 2.0;

        double first_horizon_angle;
        double max_horizon_angle = 0;
        bool rot_half = false;

        for (size_t i = 0; i < input_cloud->size(); i++)
        {
            VPoint raw_point = input_cloud->points[i];
            if (!pcl_isfinite(raw_point.x))
                continue;
            double depth = sqrt(raw_point.x * raw_point.x + raw_point.y * raw_point.y +
                                raw_point.z * raw_point.z);
            if (depth == 0)
                continue;
            double pitch = asin(raw_point.z / depth) / M_PI * 180.0;

            int ring = std::round((pitch - lidar_fov_down) / lidar_fov_resolution);
            if (ring < 0 || ring >= 16)
                continue;

            double horizon_angle = atan2(raw_point.y, -raw_point.x) / M_PI * 180.0;
            if (i == 0)
            {
                first_horizon_angle = horizon_angle;
            }
            horizon_angle -= first_horizon_angle;
            if (horizon_angle < 0)
                rot_half = true;
            if (rot_half)
                horizon_angle += 360;
            int firing = round(horizon_angle / 0.2);
            if (firing < 0 || firing >= 1824)
                continue;
            double point_time = VLP16_time_block_[firing][ring];

            RTPoint p;
            p.x = raw_point.x;
            p.y = raw_point.y;
            p.z = raw_point.z;

            p.intensity = raw_point.intensity;
            p.ring = ring;
            p.time = (float)point_time;
            output_cloud->push_back(p);

            if (max_horizon_angle < horizon_angle)
                max_horizon_angle = horizon_angle;
        }

        static double full_size = 16 * 1824;
        static double required_size = full_size * 0.8;

        // if (output_cloud->size() < required_size && max_horizon_angle < 350) {
        //   double percent = (double(output_cloud->size()) / full_size);
        //   std::cout << "points percent[" << percent
        //             << "] of /velodyne_points; horizon angle[" << max_horizon_angle
        //             << "]\n";
        // }
    }

    void VelodyneConverter::Convert(const std::string &stamp_file, const std::string &data_dir, const std::string &bag_file,
                                    const std::string &topic, const std::string &frame_id)
    {
        rosbag::Bag bag(bag_file, rosbag::bagmode::Write);
        bag.setChunkThreshold(768 * 1024);
        bag.setCompression(rosbag::compression::BZ2);

        FILE *fp = fopen(stamp_file.c_str(), "r");
        int64_t stamp;
        std::vector<int64_t> all_stamps;
        while (fscanf(fp, "%ld\n", &stamp) == 1)
        {
            all_stamps.push_back(stamp);
        }
        fclose(fp);

        size_t total = all_stamps.size();
        for (size_t i = 0; i < all_stamps.size(); ++i)
        {
            std::string st = std::to_string(all_stamps[i]);
            std::string frame_file = data_dir + "/" + st + ".bin";
            ROS_INFO("converting %s\n", frame_file.c_str());
            if (!boost::filesystem::exists(frame_file))
            {
                ROS_WARN("%s not exist\n", frame_file.c_str());
                continue;
            }
            std::ifstream file;
            file.open(frame_file, std::ios::in | std::ios::binary);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new VPointCloud);
            float angle;
            uint16_t ring;
            float time;
            while (!file.eof())
            {
                pcl::PointXYZI point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                pcl_cloud->points.push_back(point);
            }
            file.close();

            RTPointCloud::Ptr cloud_with_time(new RTPointCloud);
            RecoverVLP16Timestamp(pcl_cloud, cloud_with_time);

            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(*cloud_with_time, cloud);
            cloud.header.stamp.fromNSec(all_stamps[i]);
            cloud.header.stamp -= ros::Duration(0.1);
            cloud.header.frame_id = frame_id;
            bag.write(topic, cloud.header.stamp, cloud);
            ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
            ROS_INFO("done converting %s\n", frame_file.c_str());
            ROS_INFO("total %lu, already convert %lu, remain %lu\n", total, i + 1, total - i - 1);
        }
        bag.close();
    }

} // namespace kaist2bag