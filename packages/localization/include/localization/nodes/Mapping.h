#pragma once
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include "localization/GICP/GICP.h"
#include "localization/types/common.hpp"
#include "localization/types/QoS.hpp"

class Mapping : public rclcpp::Node
{
    static constexpr bool PRINT_BENCHMARK = false;
    static constexpr float MAP_RESOLUTION = SparseVoxel::MIN_DIST;

public:
    explicit Mapping(const std::string& node_name) : Node(node_name), mTfBuffer(this->get_clock()), mTfListener(mTfBuffer),
        mTfBroadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
    {
        declareParameters();

        /* Lookup transform */
        while (rclcpp::ok())
        {
            try
            {
                T_truck2lidar = tf2::transformToEigen(mTfBuffer.lookupTransform("LOLA", "JT128", tf2::TimePointZero)).cast<float>();
                break;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 2000, "Could not transform fork to body: %s", ex.what());
            }
        }

        GICP::Config config{};
        config.max_fitness_score = 0.1f;  // 平均意义下的点位误差尺度 10 cm
        mGicp.setConfig(config);

        mIntensityThreshold = static_cast<float>(get_parameter("intensity_threshold").as_double());
        mIntensityKeepRatio = std::clamp(static_cast<float>(get_parameter("IntensityKeepRatio").as_double()), 0.01f, 1.0f);

        initSubscribers();
        initPublisher();
        mWorker = std::thread(&Mapping::workingLoop, this);
        RCLCPP_INFO(get_logger(), "The node has been activated.");
    }

    ~Mapping() override
    {
        {
            std::lock_guard lock(mScanBufferMutex);
            mIsShutdown = true;
        }
        mTrigger.notify_one();

        saveMap();

        if (mWorker.joinable())
        {
            mWorker.join();
        }
        RCLCPP_INFO(get_logger(), "This node has been shutdown.");
    }

private:
    /* Subscribers */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarScanSub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr mWheelOdomSub;

    /* Publishers */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mProcessedScanVisPub, mVoxelMapPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mBasePosePub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mBasePosePathPub;

    /* Data Buffers */
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> mScanBuffer;

    /* Multi-thread utilities */
    bool mIsShutdown{false};
    std::thread mWorker;
    std::mutex mScanBufferMutex;
    std::condition_variable mTrigger;

    /* TF tree utilities */
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
    Eigen::Isometry3f T_truck2lidar;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;

    /* Trailer voxel map and estimated pose */
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloudMap;
    Eigen::Isometry3d mBasePose{Eigen::Isometry3d::Identity()};   // Pose of the truck
    nav_msgs::msg::Path mBasePosePath;
    GICP mGicp;
    std::vector<double> mGicpDurationsMs;
    std::size_t mGicpDurationCount{0};
    float mIntensityThreshold{-1.0f};
    float mIntensityKeepRatio{0.6f};
    bool mIntensityAnalyzed{false};

    void initSubscribers()
    {
        mLidarScanSub = create_subscription<sensor_msgs::msg::PointCloud2>(get_parameter("Topics.LidarScan").as_string(),
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr scan_msg)
                {
                    {
                       std::lock_guard lock(mScanBufferMutex);
                       while (!mScanBuffer.empty())
                       {
                           mScanBuffer.pop();
                       }
                       mScanBuffer.push(std::move(scan_msg));
                    }
                    mTrigger.notify_one();
                });
    }

    void initPublisher()
    {
        mProcessedScanVisPub = create_publisher<sensor_msgs::msg::PointCloud2>(get_parameter("Topics.RegisteredScan").as_string(), RELIABLE_LATEST);
        mVoxelMapPub = create_publisher<sensor_msgs::msg::PointCloud2>(get_parameter("Topics.CloudMap").as_string(), RELIABLE_LATEST);
        mBasePosePub = create_publisher<geometry_msgs::msg::PoseStamped>(get_parameter("Topics.BasePose").as_string(), SENSOR_LATEST);
        mBasePosePathPub = create_publisher<nav_msgs::msg::Path>(get_parameter("Topics.PosePath").as_string(), SENSOR_LATEST);
    }

    void declareParameters()
    {
        declare_parameters<std::string>("Topics", {
            {"LidarScan", "/sim_scan"},
            {"RegisteredScan", "/rigistered_scan"},
            {"CloudMap", "/cloud_map"},
            {"BasePose", "/base_pose"},
            {"PosePath", "/base_pose_path"},
        });

        declare_parameter<std::string>("MapSavingPath",
            "/home/avent/Desktop/TrailerLoadingPlacement/packages/localization/map/static_map.pcd");

        declare_parameters<double>("", {
            {"intensity_threshold", -1.0},
            {"IntensityKeepRatio", 0.99},
        });
    }

    void saveMap() const
    {
        static const std::string map_path = get_parameter("MapSavingPath").as_string();

        if (mCloudMap->size() > 100)
        {
            pcl::io::savePCDFileBinary(map_path, *mCloudMap);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to save the static map since it is almost empty!");
        }
    }

    bool alignICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_scan);

    void recordGicpDuration(double duration_ms);

    void updateVoxelMap(const pcl::PointCloud<pcl::PointXYZ>& scan_in_base_link);

    void workingLoop();
};