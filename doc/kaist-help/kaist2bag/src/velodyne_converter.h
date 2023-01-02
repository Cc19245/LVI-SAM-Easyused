//
// Created by tao on 7/25/21.
//

#ifndef SRC_VELODYNE_CONVERTER_H
#define SRC_VELODYNE_CONVERTER_H
#include <string>
#include "converter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))
typedef PointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

namespace kaist2bag {

class VelodyneConverter : public Converter {
public:
    VelodyneConverter(const std::string& dataset_dir, const std::string& save_dir,
                      const std::string& left_topic, const std::string& right_topic);
    virtual ~VelodyneConverter() = default;

    int Convert() override;

    std::string default_left_stamp_file = "sensor_data/VLP_left_stamp.csv";
    std::string default_left_data_dir = "sensor_data/VLP_left";
    std::string default_right_stamp_file = "sensor_data/VLP_right_stamp.csv";
    std::string default_right_data_dir = "sensor_data/VLP_right";

private:
    std::string left_topic_;
    std::string left_bag_name_;
    std::string right_topic_;
    std::string right_bag_name_;
    
    void RecoverVLP16Timestamp(const VPointCloud::Ptr input_cloud,
                           RTPointCloud::Ptr output_cloud);
    void Convert(const std::string& stamp_file, const std::string& data_dir,
                 const std::string& bag_file, const std::string& topic,
                 const std::string& frame_id);
};


} // namespace kaist2bag
#endif //SRC_VELODYNE_CONVERTER_H
