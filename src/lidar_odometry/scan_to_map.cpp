// scan_to_map.cpp
// Minimal LOAM-style scan-to-map optimizer (corner/surf + LM)
// Ported into LVI-SAM-Easyused/lidar_odometry/src/ for integration.
// Note: Add this file to CMakeLists.txt of the package to compile. No build is performed here.

#include "utility.h"
#include "lvi_sam/cloud_info.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

using namespace std;

class ScanToMap : public ParamServer
{
public:
    ScanToMap()
    {
        subLaserCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/feature/cloud_info", 5, &ScanToMap::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        pubOdomAftMappedROS = nh.advertise<nav_msgs::Odometry>(PROJECT_NAME + "/lidar/mapping/odometry", 1);
        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/trajectory", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/mapping/cloud_registered", 1);

        // Load tuning parameters
        nh.param<int>(PROJECT_NAME + "/scan_to_map/max_iter_count", max_iter_count, 30);
        nh.param<float>(PROJECT_NAME + "/scan_to_map/convergence_deltaR", convergence_deltaR, 1e-3);
        nh.param<float>(PROJECT_NAME + "/scan_to_map/convergence_deltaT", convergence_deltaT, 1e-3);
        nh.param<int>(PROJECT_NAME + "/scan_to_map/min_laserCloudSelNum", min_laserCloudSelNum, 50);
        nh.param<double>(PROJECT_NAME + "/scan_to_map/imu_weight", imu_weight, 0.01);

        allocateMemory();
        ROS_INFO("ScanToMap node started with tuning params: max_iter=%d, deltaR=%.6f, deltaT=%.6f, min_sel=%d, imu_w=%.4f",
                 max_iter_count, convergence_deltaR, convergence_deltaT, min_laserCloudSelNum, imu_weight);
    }

private:
    ros::Subscriber subLaserCloudInfo;
    ros::Publisher pubOdomAftMappedROS;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubRecentKeyFrame;

    // Tuning parameters
    int max_iter_count;
    float convergence_deltaR;
    float convergence_deltaT;
    int min_laserCloudSelNum;
    double imu_weight;

    lvi_sam::cloud_info cloudInfo;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec;
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;

    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    float transformTobeMapped[6];
    Eigen::Affine3f transPointAssociateToMap;

    bool isDegenerate = false;
    cv::Mat matP;

    void allocateMemory()
    {
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        for (int i = 0; i < 6; ++i) transformTobeMapped[i] = 0;
    }

    void laserCloudInfoHandler(const lvi_sam::cloud_infoConstPtr &msgIn)
    {
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        downsampleCurrentScan();

        // build trivial map from last DS (for demo) — in real use-case use keyframe map
        laserCloudCornerFromMapDS = laserCloudCornerLastDS;
        laserCloudSurfFromMapDS = laserCloudSurfLastDS;

        if (laserCloudCornerFromMapDS->empty() || laserCloudSurfFromMapDS->empty())
        {
            ROS_WARN("map empty, skipping scan2map");
            return;
        }

        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        // prepare and run scan2map optimization
        scan2MapOptimization();

        static double lastScanEndTime = 0;
        if (cloudInfo.scanEndTime <= lastScanEndTime) {
            cloudInfo.scanEndTime = lastScanEndTime + 0.1;
        }
        lastScanEndTime = cloudInfo.scanEndTime;

        publishOdometry(ros::Time(cloudInfo.scanEndTime));
    }

    void downsampleCurrentScan()
    {
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri = laserCloudCornerLastDS->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel);

            std::vector<int> pointSearchInd(5);
            std::vector<float> pointSearchSqDis(5);
            if (kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis) < 5) continue;

            cv::Mat matA1(3,3,CV_32F,cv::Scalar::all(0));
            cv::Mat matD1(1,3,CV_32F,cv::Scalar::all(0));
            cv::Mat matV1(3,3,CV_32F,cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0)
            {
                float cx=0,cy=0,cz=0;
                for (int j=0;j<5;j++){
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx/=5;cy/=5;cz/=5;
                float a11=0,a12=0,a13=0,a22=0,a23=0,a33=0;
                for (int j=0;j<5;j++){
                    float ax=laserCloudCornerFromMapDS->points[pointSearchInd[j]].x-cx;
                    float ay=laserCloudCornerFromMapDS->points[pointSearchInd[j]].y-cy;
                    float az=laserCloudCornerFromMapDS->points[pointSearchInd[j]].z-cz;
                    a11+=ax*ax; a12+=ax*ay; a13+=ax*az; a22+=ay*ay; a23+=ay*az; a33+=az*az;
                }
                a11/=5; a12/=5; a13/=5; a22/=5; a23/=5; a33/=5;
                matA1.at<float>(0,0)=a11; matA1.at<float>(0,1)=a12; matA1.at<float>(0,2)=a13;
                matA1.at<float>(1,0)=a12; matA1.at<float>(1,1)=a22; matA1.at<float>(1,2)=a23;
                matA1.at<float>(2,0)=a13; matA1.at<float>(2,1)=a23; matA1.at<float>(2,2)=a33;
                cv::eigen(matA1, matD1, matV1);
                if (matD1.at<float>(0,0) > 3*matD1.at<float>(0,1)){
                    float x0=pointSel.x, y0=pointSel.y, z0=pointSel.z;
                    float x1 = cx + 0.1*matV1.at<float>(0,0);
                    float y1 = cy + 0.1*matV1.at<float>(0,1);
                    float z1 = cz + 0.1*matV1.at<float>(0,2);
                    float x2 = cx - 0.1*matV1.at<float>(0,0);
                    float y2 = cy - 0.1*matV1.at<float>(0,1);
                    float z2 = cz - 0.1*matV1.at<float>(0,2);
                    float a012 = sqrt(((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1))*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) +
                                      ((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1))*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)) +
                                      ((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1)) );
                    float l12 = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
                    float la = ((y1-y2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) + (z1-z2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)))/a012/l12;
                    float lb = -((x1-x2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) - (z1-z2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1)))/a012/l12;
                    float lc = -((x1-x2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)) + (y1-y2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1)))/a012/l12;
                    float ld2 = a012 / l12;
                    float s = 1 - 0.9*fabs(ld2);
                    PointType coeff; coeff.x = s*la; coeff.y = s*lb; coeff.z = s*lc; coeff.intensity = s*ld2;
                    if (s>0.1){
                        laserCloudOriCornerVec[i]=pointOri;
                        coeffSelCornerVec[i]=coeff;
                        laserCloudOriCornerFlag[i]=true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri = laserCloudSurfLastDS->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel);

            std::vector<int> pointSearchInd(5);
            std::vector<float> pointSearchSqDis(5);
            if (kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis) < 5) continue;

            Eigen::Matrix<float,5,3> matA0; matA0.setZero();
            Eigen::Matrix<float,5,1> matB0; matB0.fill(-1);
            Eigen::Vector3f matX0; matX0.setZero();
            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j=0;j<5;j++){
                    matA0(j,0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j,1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j,2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                matX0 = matA0.colPivHouseholderQr().solve(matB0);
                float pa=matX0(0,0), pb=matX0(1,0), pc=matX0(2,0), pd=1;
                float ps = sqrt(pa*pa + pb*pb + pc*pc);
                pa/=ps; pb/=ps; pc/=ps; pd/=ps;
                bool planeValid=true;
                for (int j=0;j<5;j++){
                    if (fabs(pa*laserCloudSurfFromMapDS->points[pointSearchInd[j]].x + pb*laserCloudSurfFromMapDS->points[pointSearchInd[j]].y + pc*laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2){ planeValid=false; break; }
                }
                if (planeValid){
                    float pd2 = pa*pointSel.x + pb*pointSel.y + pc*pointSel.z + pd;
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z));
                    PointType coeff; coeff.x = s*pa; coeff.y = s*pb; coeff.z = s*pc; coeff.intensity = s*pd2;
                    if (s>0.1){
                        laserCloudOriSurfVec[i]=pointOri;
                        coeffSelSurfVec[i]=coeff;
                        laserCloudOriSurfFlag[i]=true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        for (int i=0;i<laserCloudCornerLastDSNum;i++){
            if (laserCloudOriCornerFlag[i]){ laserCloudOri->push_back(laserCloudOriCornerVec[i]); coeffSel->push_back(coeffSelCornerVec[i]); }
        }
        for (int i=0;i<laserCloudSurfLastDSNum;i++){
            if (laserCloudOriSurfFlag[i]){ laserCloudOri->push_back(laserCloudOriSurfVec[i]); coeffSel->push_back(coeffSelSurfVec[i]); }
        }
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < min_laserCloudSelNum) {
            ROS_WARN("Not enough optimization coeffs: %d (min: %d)", laserCloudSelNum, min_laserCloudSelNum);
            return false;
        }

        cv::Mat matA(laserCloudSelNum,6,CV_32F,cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum,1,CV_32F,cv::Scalar::all(0));

        for (int i=0;i<laserCloudSelNum;i++){
            PointType pointOri = laserCloudOri->points[i];
            PointType coeff = coeffSel->points[i];
            // lidar->camera mapping as in original
            PointType p;
            p.x = pointOri.y; p.y = pointOri.z; p.z = pointOri.x;
            PointType c; c.x = coeff.y; c.y = coeff.z; c.z = coeff.x; c.intensity = coeff.intensity;

            float arx = (crx * sry * srz * p.x + crx * crz * sry * p.y - srx * sry * p.z) * c.x + (-srx * srz * p.x - crz * srx * p.y - crx * p.z) * c.y + (crx * cry * srz * p.x + crx * cry * crz * p.y - cry * srx * p.z) * c.z;
            float ary = ((cry * srx * srz - crz * sry) * p.x + (sry * srz + cry * crz * srx) * p.y + crx * cry * p.z) * c.x + ((-cry * crz - srx * sry * srz) * p.x + (cry * srz - crz * srx * sry) * p.y - crx * sry * p.z) * c.z;
            float arz = ((crz * srx * sry - cry * srz) * p.x + (-cry * crz - srx * sry * srz) * p.y) * c.x + (crx * crz * p.x - crx * srz * p.y) * c.y + ((sry * srz + cry * crz * srx) * p.x + (crz * sry - cry * srx * srz) * p.y) * c.z;

            matA.at<float>(i,0) = arz; matA.at<float>(i,1) = arx; matA.at<float>(i,2) = ary;
            matA.at<float>(i,3) = c.z; matA.at<float>(i,4) = c.x; matA.at<float>(i,5) = c.y;
            matB.at<float>(i,0) = -c.intensity;
        }
        cv::Mat matAt, matAtA, matAtB, matX;
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        transformTobeMapped[0] += matX.at<float>(0,0);
        transformTobeMapped[1] += matX.at<float>(1,0);
        transformTobeMapped[2] += matX.at<float>(2,0);
        transformTobeMapped[3] += matX.at<float>(3,0);
        transformTobeMapped[4] += matX.at<float>(4,0);
        transformTobeMapped[5] += matX.at<float>(5,0);

        // simple convergence check (magnitude)
        float deltaR = sqrt(pow(matX.at<float>(0,0),2) + pow(matX.at<float>(1,0),2) + pow(matX.at<float>(2,0),2));
        float deltaT = sqrt(pow(matX.at<float>(3,0),2) + pow(matX.at<float>(4,0),2) + pow(matX.at<float>(5,0),2));
        if (deltaR < convergence_deltaR && deltaT < convergence_deltaT)
            return true;

        return false;
    }

    void transformUpdate()
    {
        // basic IMU-assisted small fusion for roll/pitch if available
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imu_weight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imu_weight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        // apply simple constraints
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;
        return value;
    }

    void scan2MapOptimization()
    {
        if (laserCloudCornerFromMapDS->empty() || laserCloudSurfFromMapDS->empty()) return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < max_iter_count; iterCount++)
            {
                laserCloudOri->clear(); coeffSel->clear();
                cornerOptimization();
                surfOptimization();
                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;
            }

            transformUpdate();
        }
        else
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        // identity for now (no global transform applied)
        po->x = pi->x; po->y = pi->y; po->z = pi->z; po->intensity = pi->intensity;
    }

    void publishOdometry(const ros::Time &stamp)
    {
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = stamp;
        laserOdometryROS.header.frame_id = "odom";
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubOdomAftMappedROS.publish(laserOdometryROS);

        // publish recent key frame (registered)
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            *cloudOut += *laserCloudCornerLastDS;
            *cloudOut += *laserCloudSurfLastDS;
            publishCloud(&pubRecentKeyFrame, cloudOut, ros::Time::now(), "odom");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_map");
    ScanToMap node;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}
