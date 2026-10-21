#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include "free_space_detection/types/common.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FreeSpaceDetect final : public rclcpp::Node
{
    const std::string global_frame_id = "map";

    static geometry_msgs::msg::Quaternion toQuaternionMsg(const double yaw)
    {
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0.0, 0.0, yaw);
        geometry_msgs::msg::Quaternion geom_quat;
        tf2::convert(tf2_quat, geom_quat);
        return geom_quat;
    }

public:
    explicit FreeSpaceDetect(const std::string& name, const rclcpp::NodeOptions& options) : rclcpp::Node(name, options),
        mTfBuffer(this->get_clock()), mTfListener(mTfBuffer)
    {
        initSubscriptions();
        initPublishers();
        // this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        mWorker = std::thread(&FreeSpaceDetect::workerLoop, this);
        RCLCPP_INFO(get_logger(), "The node has been activated.");
    }

    ~FreeSpaceDetect() override
    {
        {
            std::unique_lock lock(mBufferMutex);
            mIsShutdown = true;
        }
        mTriggerEvent.notify_one();
        if (mWorker.joinable())
        {
            mWorker.join();
        }
        RCLCPP_INFO(get_logger(), "The node has been shutdown.");
    }

private:
    /* Received Data Buffer */
    bool mHasGoal{false}, mIsShutdown{false};
    std::mutex mBufferMutex;
    std::condition_variable mTriggerEvent;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> mCloudBuffer;
    std::thread mWorker;

    geometry_msgs::msg::PoseStamped mGoalMsg;

    /** Subscribers **/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mCloudSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalSub;

    // /*** Synchronized Subsribers ***/
    // using CloudMsg = sensor_msgs::msg::PointCloud2;
    // using JointStatesMsg = sensor_msgs::msg::JointState;

    /** Publishers **/
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mSlotPosePub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mVisualizationPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDebugImgPub;

    /* TF */
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    /* Algorithm Logic: History Data */
    struct
    {
        Eigen::Vector3f trailer_pose;
        Eigen::Vector3f trailer_dimensions;
        Eigen::Vector3f ramp_position;
        Eigen::Vector2f ramp_size;
        float ramp_tilt{};

        void reset()
        {
            trailer_pose.setZero();
            trailer_dimensions.setZero();
            ramp_position.setZero();
            ramp_size.setZero();
            ramp_tilt = 0.0f;
        }
    } mHistoryBuffer;
    std::size_t mHistoryCount{};
    std::vector<Eigen::Vector2f> mPlacementZones;

    void initSubscriptions();

    void initPublishers();

    void publishDebugImg(const cv::Mat& img) const
    {
        if (img.empty())
        {
            RCLCPP_WARN(get_logger(), "Cannot publish an empty debug image!");
            return;
        }

        cv::Mat rgb_img;
        switch (img.type())
        {
            case CV_8UC1:
                cv::cvtColor(img, rgb_img, cv::COLOR_GRAY2RGB);
                break;
            case CV_8UC3:
                cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);
                break;
            case CV_8UC4:
                cv::cvtColor(img, rgb_img, cv::COLOR_BGRA2RGB);
                break;
            default:
                RCLCPP_WARN(get_logger(), "Unsupported debug image type: %d", img.type());
                return;
        }

        cv_bridge::CvImage img_bridge;
        img_bridge.header.stamp = this->now(); // Optional: set a timestamp
        img_bridge.header.frame_id = "camera_frame";
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8; // e.g., "bgr8"
        img_bridge.image = std::move(rgb_img);
        sensor_msgs::msg::Image img_msg;
        img_bridge.toImageMsg(img_msg);
        mDebugImgPub->publish(img_msg);
    }

    /* Sub detection modules */
    void reset()
    {
        mPlacementZones.clear();
        mHistoryBuffer.reset();
        mHistoryCount = 0;
    }

    bool locateTrailer(const RawCloud::Ptr& cloud, const RawCloud::Ptr& trailer_cloud, Eigen::Vector3f& trailer_pose, ROI& trailer_bbox) const;

    bool estimateRampPose(const RawCloud::Ptr& cloud, Eigen::Vector3f& ramp_position, float& ramp_tilt, Eigen::Vector2f& ramp_size) const;

    bool planPlacementZone(const Eigen::Vector3f& trailer_pose, const Eigen::Vector2f& trailer_size, const Eigen::Vector2f& load_size);

    bool locateAvailableArea(const RawCloud::Ptr& trailer_cloud, const Eigen::Vector3f& load_size, Eigen::Vector3f& slot_position) const;

    bool validateSlot(const RawCloud::Ptr& cloud, const Eigen::Vector3f& slot_position, const Eigen::Vector3f& load_dimensions) const;

    void workerLoop();
};