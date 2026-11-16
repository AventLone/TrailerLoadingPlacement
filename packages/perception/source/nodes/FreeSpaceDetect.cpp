#include "free_space_detection/nodes/FreeSpaceDetect.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "free_space_detection/tools/filter_2d.h"
#include "free_space_detection/tools/filter_3d.h"
#include "free_space_detection/tools/feature_detect_3d.h"
#include "free_space_detection/tools/OrthographicProjector.hpp"
#include "free_space_detection/types/QoS.hpp"

void FreeSpaceDetect::initSubscriptions()
{
    const std::string param_name = "TopicName.Perception.SemanticCloud";
    this->declare_parameter(param_name, "/perception/semantic_cloud");
    const std::string semantic_cloud_topic = this->get_parameter(param_name).as_string();
    mGoalSub = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::ServicesQoS(),
                [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal_msg) -> void
                    {
                        RCLCPP_INFO(get_logger(), "Received a goal...");
                        this->mGoalMsg = *goal_msg;
                        {
                            std::lock_guard lock(mBufferMutex);
                            this->mHasGoal = true;
                        }
                        reset();
                        this->mTriggerEvent.notify_one();
                    });

    mCloudSub = create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_map", free_space_detection::qos::RELIABLE_LATEST,
                                                                   [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
                                                                         {
                                                                             std::lock_guard lock(mBufferMutex);
                                                                             while (!mCloudBuffer.empty())
                                                                             {
                                                                                mCloudBuffer.pop();
                                                                             }
                                                                             mCloudBuffer.push(std::move(msg));
                                                                             this->mTriggerEvent.notify_one();
                                                                         });
}

void FreeSpaceDetect::initPublishers()
{
    const std::string params_prefix = "TopicName.Perception.Target";
    this->declare_parameters<std::string>(params_prefix, {{"Pose", "/perception/target/pose"},
                                                         {"BBox", "/perception/visualization/cubes"}});
    std::map<std::string, std::string> target_topics;
    if (!this->get_parameters<std::string>(params_prefix, target_topics))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameters, target topic names!");
    }
    mVisualizationPub = create_publisher<visualization_msgs::msg::MarkerArray>(target_topics["BBox"], rclcpp::SensorDataQoS());
    mSlotPosePub = create_publisher<geometry_msgs::msg::PoseStamped>("slot_pose", rclcpp::ServicesQoS());
    mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("debug_img", rclcpp::SensorDataQoS());
    mResultPub = create_publisher<free_space_detection::msg::PerceptionResult>("detect_result", free_space_detection::qos::RELIABLE_LATEST);
}

void FreeSpaceDetect::workerLoop()
{
    constexpr std::string_view ns = "visualization";
    constexpr std::string_view FRAME_ID = "map";
    const auto LOAD_SIZE(Eigen::Vector3f(1.2f, 1.0f, 1.5f));

    free_space_detection::msg::PerceptionResult perception_result;

    rclcpp::WallRate publish_rate(5.0);   // rclcpp::Rate publish_rate(5.0, this->get_clock());
    while (rclcpp::ok())
    {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;
        {
            std::unique_lock lock(mBufferMutex);
            mTriggerEvent.wait(lock, [this]() -> bool
                                     {
                                         return (mHasGoal && !mCloudBuffer.empty()) || mIsShutdown;
                                     });
            if (mIsShutdown)
            {
                break;
            }
            cloud_msg = std::move(mCloudBuffer.front());
            mCloudBuffer.pop();
        }

        perception_result.success = false;
        perception_result.header.stamp = cloud_msg->header.stamp;

        visualization_msgs::msg::MarkerArray visualization_msg;

        auto cloud = std::make_shared<RawCloud>();
        pcl::fromROSMsg(*cloud_msg, *cloud);

        constexpr ROI trailer_roi{-20.0f, 3.0f, -3.0f, 3.0f, -1.0f, 5.0f};

        /* Visualize slot_space_roi */
        tf2::Quaternion tf_q;
        tf2::fromMsg(mGoalMsg.pose.orientation, tf_q); // Convert ROS msg → tf2 quaternion

        // 3. Convert quaternion to roll/pitch/yaw (Euler angles)
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw); // Order: X(roll), Y(pitch), Z(yaw)
        Eigen::Isometry2f T_1(Eigen::Isometry2f::Identity()), T_2(Eigen::Isometry2f::Identity());
        T_1.rotate(yaw);
        T_1.pretranslate(Eigen::Vector2f(mGoalMsg.pose.position.x, mGoalMsg.pose.position.y));
        T_2.translate(Eigen::Vector2f(-0.5f * 17.0f, 0.0f));
        Eigen::Isometry2f T_3 = T_1 * T_2;
        const Eigen::Vector3d trailer_roi_position(T_3.translation()[0], T_3.translation()[1],
                                                   0.5f * (trailer_roi.min_z + trailer_roi.max_z));
        const Eigen::Vector3d trailer_roi_scale(trailer_roi.max_x - trailer_roi.min_x,
                                                trailer_roi.max_y - trailer_roi.min_y,
                                                trailer_roi.max_z - trailer_roi.min_z);
        const Eigen::Vector4d trailer_roi_color(0.8, 1.0, 0.8, 0.2);
        auto trailer_roi_marker = makeMarker(FRAME_ID, ns, 1, trailer_roi_position, tf_q, trailer_roi_scale, trailer_roi_color);
        visualization_msg.markers.push_back(trailer_roi_marker);

        Eigen::Isometry3f goal_pose;
        {
            Eigen::Isometry3d temp{};
            tf2::fromMsg(mGoalMsg.pose, temp);
            goal_pose = temp.cast<float>();
        }

        /*------ Stage 2. Slot pose estimate ------*/
        auto rough_trailer_cloud = std::make_shared<RawCloud>();
        filter3d::getCloud(*cloud, rough_trailer_cloud, nullptr, goal_pose, trailer_roi);

        if (rough_trailer_cloud->size() < 10)
        {
            RCLCPP_ERROR(get_logger(), "slot_space_cloud has too few points!");
            mResultPub->publish(perception_result);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        const auto refined_trailer_cloud = std::make_shared<RawCloud>();
        Eigen::Vector3f trailer_pose;
        ROI trailer_bbox{};
        if (!locateTrailer(rough_trailer_cloud, refined_trailer_cloud, trailer_pose, trailer_bbox))
        {
            RCLCPP_ERROR(get_logger(), "Failed to locate the trailer or the ramp!");
            mResultPub->publish(perception_result);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        const auto trailer_dimensions = Eigen::Vector3f(trailer_bbox.max_x - trailer_bbox.min_x,
                                                        trailer_bbox.max_y - trailer_bbox.min_y,
                                                        trailer_bbox.max_z - trailer_bbox.min_z);

        tf2::Quaternion trailer_orientation;
        trailer_orientation.setRPY(0.0, 0.0, trailer_pose.z());
        const Eigen::Vector3f center_rotated(0.5f * (trailer_bbox.min_x + trailer_bbox.max_x),
                                             0.5f * (trailer_bbox.min_y + trailer_bbox.max_y),
                                             0.5f * (trailer_bbox.min_z + trailer_bbox.max_z));
        const Eigen::Vector2f center_in_frame = Eigen::Rotation2Df(trailer_pose.z()) * center_rotated.head<2>();
        const Eigen::Vector3d trailer_position(center_in_frame.x(), center_in_frame.y(), center_rotated.z());
        const Eigen::Vector4d trailer_color(0.9, 0.9, 0.9, 0.3);
        auto trailer_marker = makeMarker(FRAME_ID, ns, 2, trailer_position, trailer_orientation, trailer_dimensions.cast<double>(), trailer_color);
        visualization_msg.markers.push_back(trailer_marker);

        const auto rough_ramp_cloud = std::make_shared<RawCloud>();
        ROI ramp_roi{};
        ramp_roi.min_x = trailer_pose.x();
        ramp_roi.min_x = trailer_pose.x() + 1.0f;
        ramp_roi.min_y = -0.5f * trailer_dimensions.y();
        ramp_roi.max_y = 0.5f * trailer_dimensions.y();
        ramp_roi.min_z = -1.0f;
        ramp_roi.max_z = 1.0f;
        filter3d::getCloud(*rough_trailer_cloud, rough_ramp_cloud, nullptr, ramp_roi);
        Eigen::Vector3f ramp_position;
        float ramp_tilt;
        Eigen::Vector2f ramp_size;
        if (estimateRampPose(rough_ramp_cloud, ramp_position, ramp_tilt, ramp_size))
        {

            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // continue;
            tf2::Quaternion ramp_orientation;
            ramp_orientation.setRPY(0.0, -static_cast<double>(ramp_tilt), 0.0);
            const Eigen::Vector3d ramp_marker_position(ramp_position.x(), ramp_position.y(), ramp_position.z());
            const Eigen::Vector3d ramp_marker_scale(ramp_size.x(), ramp_size.y(), 0.04);
            const Eigen::Vector4d ramp_marker_color(0.1, 0.75, 1.0, 0.8);
            auto ramp_marker = makeMarker(FRAME_ID, ns, 3, ramp_marker_position, ramp_orientation, ramp_marker_scale, ramp_marker_color);
            visualization_msg.markers.push_back(ramp_marker);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to estimate pose of the ramp!");
        }

        if (mPlacementZones.empty())
        {
            if (!planPlacementZone(trailer_pose, trailer_dimensions.head<2>() - Eigen::Vector2f(0.01f, 0.01f),
                                   {LOAD_SIZE.x() + 0.01f, LOAD_SIZE.y() + 0.01f}))
            {
                RCLCPP_ERROR(get_logger(), "Failed to plan placement zones!");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                mResultPub->publish(perception_result);
                continue;
            }
        }

        for (std::size_t index = 0; index < mPlacementZones.size(); ++index)
        {
            constexpr float placement_marker_height = 0.03f;
            const Eigen::Vector2f position_in_map = Eigen::Rotation2Df(trailer_pose.z()) * mPlacementZones[index];
            const Eigen::Vector3d placement_position(position_in_map.x(), position_in_map.y(),
                                                     trailer_bbox.min_z + 0.2f);
            const Eigen::Vector3d placement_scale(LOAD_SIZE.x(), LOAD_SIZE.y(), placement_marker_height);
            std::mt19937 color_generator(index);
            std::uniform_real_distribution color_distribution(0.2f, 1.0f);
            const double red = color_distribution(color_generator);
            const double green = color_distribution(color_generator);
            const double blue = color_distribution(color_generator);
            const Eigen::Vector4d placement_color(red, green, blue, 0.55);
            auto placement_marker = makeMarker(FRAME_ID, ns, static_cast<int>(index) + 10, placement_position,
                                               trailer_orientation, placement_scale, placement_color);
            visualization_msg.markers.push_back(placement_marker);

            const Eigen::Vector3d center_scale(0.08, 0.08, 0.08);
            const Eigen::Vector4d center_color(1.0, 1.0, 1.0, 1.0);
            auto center_marker = makeMarker(FRAME_ID, "placement_centers", static_cast<int>(index) + 10,
                                            placement_position, trailer_orientation, center_scale, center_color,
                                            visualization_msgs::msg::Marker::SPHERE);
            center_marker.header.stamp = placement_marker.header.stamp;
            visualization_msg.markers.push_back(center_marker);

            Eigen::Vector3d label_position = placement_position;
            label_position.z() += 0.15;
            tf2::Quaternion label_orientation;
            label_orientation.setRPY(0.0, 0.0, 0.0);
            const Eigen::Vector3d label_scale(0.0, 0.0, 0.3);
            auto label_marker = makeMarker(FRAME_ID, "placement_labels", static_cast<int>(index) + 10,
                                           label_position, label_orientation, label_scale, center_color,
                                           visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label_marker.header.stamp = placement_marker.header.stamp;
            label_marker.text = std::to_string(index + 1);
            visualization_msg.markers.push_back(label_marker);
        }

        visualization_msgs::msg::Marker available_marker;
        available_marker.header.frame_id = FRAME_ID;
        available_marker.header.stamp = this->now();
        available_marker.ns = "available_area";
        available_marker.id = 0;
        available_marker.type = visualization_msgs::msg::Marker::CUBE;
        available_marker.action = visualization_msgs::msg::Marker::DELETE;
        if (Eigen::Vector3f slot_position; locateAvailableArea(refined_trailer_cloud, LOAD_SIZE, slot_position))
        {
            const Eigen::Vector2f position_in_map = Eigen::Rotation2Df(trailer_pose.z()) * slot_position.head<2>();
            const Eigen::Vector3d available_position(position_in_map.x(), position_in_map.y(), trailer_bbox.min_z + 0.5f * LOAD_SIZE.z());
            const Eigen::Vector4d available_color(0.0, 1.0, 0.0, 0.6);
            available_marker = makeMarker(FRAME_ID, "available_area", 0, available_position, trailer_orientation,
                                          LOAD_SIZE.cast<double>(), available_color);

            perception_result.slot_position.x = slot_position.x();
            perception_result.slot_position.y = slot_position.y();
            perception_result.slot_position.z = slot_position.z();
            mResultPub->publish(perception_result);
        }
        visualization_msg.markers.push_back(available_marker);

        mResultPub->publish(perception_result);
        mVisualizationPub->publish(visualization_msg);

        publish_rate.sleep();
    }
}

bool FreeSpaceDetect::exploreTrailer(const RawCloud::Ptr& cloud)
{

}

bool FreeSpaceDetect::locateTrailer(const RawCloud::Ptr& cloud, const RawCloud::Ptr& trailer_cloud, Eigen::Vector3f& trailer_pose, ROI& trailer_bbox) const
{
    if (cloud->size() < 100)
    {
        RCLCPP_WARN(get_logger(), "There's too few points to locate the trailer!");
        return false;
    }

    const auto cloud_without_floor_and_ceil = std::make_shared<RawCloud>();
    cloud_without_floor_and_ceil->reserve(cloud->size());
    for (const auto& point : cloud->points)
    {
        if (point.z > 0.2f && point.z < 1.7f)
        {
            cloud_without_floor_and_ceil->emplace_back(point.x, point.y, point.z);
        }
    }

    if (cloud_without_floor_and_ceil->size() < 100)
    {
        RCLCPP_WARN(get_logger(), "cloud_without_floor has too few points!");
        return false;
    }

    /* Get the top-down view image */
    constexpr float resolution = 0.05f;
    // Trailer side walls must be 1-5 m apart; convert the limits to image pixels.
    constexpr float min_side_separation = 1.0f / resolution;
    constexpr float max_side_separation = 5.0f / resolution;
    OrthographicProjector<pcl::PointXYZ> projector(View::TOP, resolution);
    projector.setCloud(cloud_without_floor_and_ceil);

    cv::Mat top_down_img = projector.projection();
    cv::Mat debug_img;
    cv::cvtColor(top_down_img, debug_img, cv::COLOR_GRAY2BGR);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(top_down_img, lines, 1.0, CV_PI / 180.0, 60, 40.0, 30.0);

    constexpr int boundary_bin_width = 10;
    constexpr float boundary_tolerance = 0.15f / resolution;
    std::vector<cv::Vec3f> boundary_samples;
    for (int bin_start = 0; bin_start < top_down_img.cols; bin_start += boundary_bin_width)
    {
        std::vector<float> occupied_columns, upper_rows, lower_rows;
        for (int col = bin_start; col < std::min(bin_start + boundary_bin_width, top_down_img.cols); ++col)
        {
            int upper_row = -1;
            int lower_row = -1;
            for (int row = 0; row < top_down_img.rows; ++row)
            {
                if (top_down_img.at<uchar>(row, col) != 0)
                {
                    if (upper_row < 0)
                    {
                        upper_row = row;
                    }
                    lower_row = row;
                }
            }
            if (upper_row >= 0)
            {
                occupied_columns.push_back(static_cast<float>(col));
                upper_rows.push_back(static_cast<float>(upper_row));
                lower_rows.push_back(static_cast<float>(lower_row));
            }
        }
        if (occupied_columns.size() < 3)
        {
            continue;
        }
        std::ranges::sort(upper_rows);
        std::ranges::sort(lower_rows);
        const std::size_t middle = occupied_columns.size() / 2;
        boundary_samples.emplace_back(occupied_columns[middle], upper_rows[middle], lower_rows[middle]);
    }

    std::vector<int> candidate_indices;
    for (int index = 0; index < static_cast<int>(lines.size()); ++index)
    {
        const cv::Vec4i& line = lines[index];

        if (const double angle = std::atan2(static_cast<double>(line[3] - line[1]), static_cast<double>(line[2] - line[0])) * 180.0 /
                                 CV_PI; std::abs(angle) <= 30.0)
        {
            candidate_indices.push_back(index);
        }
    }

    int best_first = -1;
    int best_second = -1;
    double best_score = -1.0;

    for (std::size_t i = 0; i < candidate_indices.size(); ++i)
    {
        const cv::Vec4i& first_line = lines[candidate_indices[i]];
        const cv::Point2f first_start(static_cast<float>(first_line[0]), static_cast<float>(first_line[1]));
        const cv::Point2f first_end(static_cast<float>(first_line[2]), static_cast<float>(first_line[3]));
        const cv::Point2f first_direction = first_end - first_start;
        const double first_length = cv::norm(first_direction);
        const cv::Point2f first_unit = first_direction / static_cast<float>(first_length);
        const cv::Point2f first_midpoint = (first_start + first_end) * 0.5f;

        for (std::size_t j = i + 1; j < candidate_indices.size(); ++j)
        {
            const cv::Vec4i& second_line = lines[candidate_indices[j]];
            const cv::Point2f second_start(static_cast<float>(second_line[0]), static_cast<float>(second_line[1]));
            const cv::Point2f second_end(static_cast<float>(second_line[2]), static_cast<float>(second_line[3]));
            const cv::Point2f second_direction = second_end - second_start;
            const double second_length = cv::norm(second_direction);

            if (const cv::Point2f second_unit = second_direction / static_cast<float>(second_length);
                std::abs(first_unit.dot(second_unit)) < std::cos(10.0 * CV_PI / 180.0))
            {
                continue;
            }

            const cv::Point2f second_midpoint = (second_start + second_end) * 0.5f;
            const cv::Point2f midpoint_delta = second_midpoint - first_midpoint;

            if (const double separation = std::abs(first_unit.x * midpoint_delta.y - first_unit.y * midpoint_delta.x);
                separation < min_side_separation || separation > max_side_separation)
            {
                continue;
            }

            const double first_min = std::min(first_start.dot(first_unit), first_end.dot(first_unit));
            const double first_max = std::max(first_start.dot(first_unit), first_end.dot(first_unit));
            const double second_min = std::min(second_start.dot(first_unit), second_end.dot(first_unit));
            const double second_max = std::max(second_start.dot(first_unit), second_end.dot(first_unit));
            const double overlap = std::max(0.0, std::min(first_max, second_max) - std::max(first_min, second_min));

            int sampled_bins = 0;
            int first_support = 0;
            int second_support = 0;
            int outside_bins = 0;
            const float min_col = std::min({first_start.x, first_end.x, second_start.x, second_end.x});
            const float max_col = std::max({first_start.x, first_end.x, second_start.x, second_end.x});
            for (const auto& boundary : boundary_samples)
            {
                if (boundary[0] < min_col || boundary[0] > max_col)
                {
                    continue;
                }
                const float first_row = first_start.y + (boundary[0] - first_start.x) * first_direction.y / first_direction.x;
                const float second_row = second_start.y + (boundary[0] - second_start.x) * second_direction.y / second_direction.x;
                const bool first_is_upper = first_row < second_row;
                const float upper_row = std::min(first_row, second_row);
                const float lower_row = std::max(first_row, second_row);
                ++sampled_bins;
                first_support += std::abs(first_row - boundary[first_is_upper ? 1 : 2]) <= boundary_tolerance;
                second_support += std::abs(second_row - boundary[first_is_upper ? 2 : 1]) <= boundary_tolerance;
                outside_bins += boundary[1] < upper_row - boundary_tolerance || boundary[2] > lower_row + boundary_tolerance;
            }

            if (sampled_bins < 3 || first_support < 2 || second_support < 2 || outside_bins > std::max(1, sampled_bins / 5))
            {
                continue;
            }

            const double boundary_support = static_cast<double>(first_support + second_support) / (2 * sampled_bins);
            if (const double score = (std::min(first_length, second_length) + 0.5 * overlap) * boundary_support;
                score > best_score)
            {
                best_score = score;
                best_first = candidate_indices[i];
                best_second = candidate_indices[j];
            }
        }
    }

    for (const auto candidate : candidate_indices)
    {
        const auto& line = lines[candidate];
        cv::line(debug_img, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    if (best_first < 0)
    {
        RCLCPP_WARN(get_logger(), "Failed to find the trailer!");
        return false;
    }

    std::vector<cv::Mat> side_masks(2);
    const auto line_indices = std::vector{best_first, best_second};
    for (std::size_t i = 0; i < 2; ++i)
    {
        side_masks[i] = cv::Mat::zeros(top_down_img.size(), CV_8UC1);
        const cv::Vec4i& line = lines[line_indices[i]];
        cv::line(debug_img, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        const float slope = static_cast<float>(line[3] - line[1]) / static_cast<float>(line[2] - line[0]);
        cv::Point mask_start(0, cvRound(static_cast<float>(line[1]) - slope * static_cast<float>(line[0])));
        cv::Point mask_end(top_down_img.cols - 1,
                           cvRound(static_cast<float>(line[1]) + slope * static_cast<float>(top_down_img.cols - 1 - line[0])));
        if (cv::clipLine(top_down_img.size(), mask_start, mask_end))
        {
            cv::line(side_masks[i], mask_start, mask_end, cv::Scalar(255), 5, cv::LINE_AA);
        }
    }

    const cv::Vec4i& first_line = lines[best_first];
    cv::Point2f first_start(static_cast<float>(first_line[0]), static_cast<float>(first_line[1]));
    cv::Point2f first_end(static_cast<float>(first_line[2]), static_cast<float>(first_line[3]));

    const cv::Vec4i& second_line = lines[best_second];
    cv::Point2f second_start(static_cast<float>(second_line[0]), static_cast<float>(second_line[1]));
    cv::Point2f second_end(static_cast<float>(second_line[2]), static_cast<float>(second_line[3]));

    // Orient both side lines in the same direction before comparing their ends.
    if (const cv::Point2f first_direction = first_end - first_start; first_direction.dot(second_end - second_start) < 0.0f)
    {
        std::swap(second_start, second_end);
    }

    cv::line(debug_img, first_start, second_start, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);

    /* Get 2 side wall point cloud */
    const auto side_wall_1 = projector.extractCloud(side_masks[0]);
    const auto side_wall_2 = projector.extractCloud(side_masks[1]);

    const auto refine_wall = [](const RawCloud& wall_cloud, const cv::Vec4i& image_line, RawCloud& refined_cloud)
        {
            if (wall_cloud.size() < 10)
            {
                return false;
            }
            const Eigen::Vector3f expected_normal(static_cast<float>(image_line[3] - image_line[1]),
                                                  static_cast<float>(image_line[2] - image_line[0]), 0.0f);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.04);
            seg.setAxis(expected_normal.normalized());
            seg.setEpsAngle(10.0 * CV_PI / 180.0);
            seg.setInputCloud(wall_cloud.makeShared());
            pcl::PointIndices inliers;
            pcl::ModelCoefficients coefficients;
            seg.segment(inliers, coefficients);
            if (inliers.indices.size() < 10 || coefficients.values.size() != 4)
            {
                return false;
            }
            feature3d::getCloud(wall_cloud, refined_cloud, inliers.indices);
            return true;
        };

    /* Get 2 refined side wall point cloud */
    pcl::PointCloud<pcl::PointXYZ> refined_side_wall_1, refined_side_wall_2;
    if (!refine_wall(side_wall_1, first_line, refined_side_wall_1) || !refine_wall(side_wall_2, second_line, refined_side_wall_2))
    {
        RCLCPP_WARN(get_logger(), "Insufficient side wall points after refinement!");
        return false;
    }

    auto side_wall_1_plane = feature3d::fitPlaneByPCA(refined_side_wall_1, 2);
    auto side_wall_2_plane = feature3d::fitPlaneByPCA(refined_side_wall_2, 2);
    if (!side_wall_1_plane.allFinite() || !side_wall_2_plane.allFinite() || side_wall_1_plane.head<3>().norm() < 1.0e-6f ||
        side_wall_2_plane.head<3>().norm() < 1.0e-6f)
    {
        RCLCPP_WARN(get_logger(), "Invalid side wall plane coefficients!");
        return false;
    }
    side_wall_1_plane /= side_wall_1_plane.head<3>().norm();
    side_wall_2_plane /= side_wall_2_plane.head<3>().norm();
    if (side_wall_1_plane.head<3>().dot(side_wall_2_plane.head<3>()) < 0.0f)
    {
        side_wall_2_plane = -side_wall_2_plane;
    }
    Eigen::Vector4f mid_plane = 0.5f * (side_wall_1_plane + side_wall_2_plane);
    if (mid_plane[1] < 0.0f)
    {
        mid_plane = -mid_plane;
    }
    const float angle = std::atan2(-mid_plane[0], mid_plane[1]);

    // 1. Rotation transformation from truck frame to rotated frame
    Eigen::Isometry3f T_rot = Eigen::Isometry3f::Identity();
    T_rot.rotate(Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(refined_side_wall_1, refined_side_wall_1, T_rot);
    pcl::transformPointCloud(refined_side_wall_2, refined_side_wall_2, T_rot);

    const Eigen::Vector3f normal_1 = T_rot.rotation() * side_wall_1_plane.head<3>();
    const Eigen::Vector3f normal_2 = T_rot.rotation() * side_wall_2_plane.head<3>();
    const float wall_1_y = -side_wall_1_plane[3] / normal_1.y();
    const float wall_2_y = -side_wall_2_plane[3] / normal_2.y();

    const bool wall_1_is_left = wall_1_y < wall_2_y;

    pcl::PointXYZ wall_1_min, wall_1_max, wall_2_min, wall_2_max;
    pcl::getMinMax3D(refined_side_wall_1, wall_1_min, wall_1_max);
    pcl::getMinMax3D(refined_side_wall_2, wall_2_min, wall_2_max);

    trailer_bbox.min_y = wall_1_is_left ? wall_1_max.y : wall_2_max.y;
    trailer_bbox.max_y = wall_1_is_left ? wall_2_min.y : wall_1_min.y;

    pcl::PointCloud<pcl::PointXYZ> side_walls = refined_side_wall_1 + refined_side_wall_2;

    Eigen::Vector4f rotated_mid_plane;
    rotated_mid_plane.head<3>() = T_rot.rotation() * mid_plane.head<3>();
    rotated_mid_plane[3] = mid_plane[3];

    /* Find out the max x of `side_walls` */
    float max_x = std::numeric_limits<float>::lowest();
    for (const auto& point : side_walls)
    {
        max_x = std::max(max_x, point.x);
    }
    const float y_offset = -rotated_mid_plane[3] / rotated_mid_plane[1];

    /* Get the pose of the trailer */
    trailer_pose = Eigen::Vector3f(max_x, y_offset, angle);

    /* Get the dimensions of the trailer */
    ROI trailer_roi = ROI::getBBox(side_walls);
    trailer_roi.min_x -= 9.9f;
    trailer_roi.min_z = -1.1f;
    trailer_roi.max_z += 3.1f;

    RawCloud non_angular_cloud;
    pcl::transformPointCloud(*cloud, non_angular_cloud, T_rot);
    filter3d::getCloud(non_angular_cloud, trailer_cloud, nullptr, trailer_roi);

    pcl::PointXYZ trailer_min, trailer_max;
    pcl::getMinMax3D(*trailer_cloud, trailer_min, trailer_max);
    trailer_bbox.min_z = trailer_min.z;
    trailer_bbox.max_z = trailer_max.z;

    const auto back_wall = std::make_shared<RawCloud>();
    filter3d::getCloud(*trailer_cloud, back_wall, nullptr, {.min_x = trailer_min.x, .max_x = trailer_min.x + 0.2f,
                       .min_y = trailer_bbox.min_y + 0.1f, .max_y = trailer_bbox.max_y - 0.1f,
                       .min_z = 0.5f * (trailer_bbox.min_z + trailer_bbox.max_z), .max_z = trailer_bbox.max_z - 0.1f});
    pcl::PointXYZ back_wall_min, back_wall_max;
    pcl::getMinMax3D(*back_wall, back_wall_min, back_wall_max);
    trailer_bbox.min_x = back_wall_max.x;
    trailer_bbox.max_x = max_x;

    if (trailer_cloud->size() < 100)
    {
        RCLCPP_WARN(get_logger(), "Failed to find the trailer!");
        return false;
    }

    return true;
}

bool FreeSpaceDetect::estimateRampPose(const RawCloud::Ptr& cloud, Eigen::Vector3f& ramp_position, float& ramp_tilt, Eigen::Vector2f& ramp_size) const
{
    if (cloud->size() < 100)
    {
        RCLCPP_WARN(get_logger(), "There are too few points for ramp detection!");
        return false;
    }
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE); // ramp 与 y 轴平行
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setAxis(Eigen::Vector3f(0.0f, 1.0f, 0.0f)); // Y 轴
    seg.setEpsAngle(2.0 * M_PI / 180.0); // 允许偏差 2°
    seg.setInputCloud(cloud);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    seg.segment(inliers, coefficients);

    if (inliers.indices.size() < 30 || coefficients.values.size() < 4)
    {
        RCLCPP_WARN(get_logger(), "Failed to fit the ramp plane!");
        return false;
    }

    Eigen::Vector3f normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    const float normal_length = normal.norm();
    if (normal_length < 1.0e-6f)
    {
        RCLCPP_WARN(get_logger(), "Ramp plane has an invalid normal!");
        return false;
    }
    normal /= normal_length;
    const float plane_offset = coefficients.values[3] / normal_length;

    Eigen::Vector3f ramp_direction = Eigen::Vector3f::UnitY().cross(normal);
    const float direction_length = ramp_direction.norm();
    if (direction_length < 1.0e-6f)
    {
        RCLCPP_WARN(get_logger(), "Ramp direction is degenerate!");
        return false;
    }
    ramp_direction /= direction_length;
    if (ramp_direction.x() < 0.0f)
    {
        ramp_direction = -ramp_direction;
    }

    float min_ramp = std::numeric_limits<float>::max();
    float max_ramp = std::numeric_limits<float>::lowest();
    float min_width = std::numeric_limits<float>::max();
    float max_width = std::numeric_limits<float>::lowest();
    for (const int index : inliers.indices)
    {
        const auto& point = cloud->points[static_cast<std::size_t>(index)];
        const Eigen::Vector3f position(point.x, point.y, point.z);
        const float ramp_coordinate = ramp_direction.dot(position);
        min_ramp = std::min(min_ramp, ramp_coordinate);
        max_ramp = std::max(max_ramp, ramp_coordinate);
        min_width = std::min(min_width, point.y);
        max_width = std::max(max_width, point.y);
    }

    const float ramp_length = max_ramp - min_ramp;
    const float ramp_width = max_width - min_width;
    if (ramp_length <= 0.0f || ramp_width <= 0.0f)
    {
        RCLCPP_WARN(get_logger(), "Ramp plane has invalid dimensions!");
        return false;
    }

    const float ramp_coordinate = 0.5f * (min_ramp + max_ramp);
    const float width_coordinate = 0.5f * (min_width + max_width);
    ramp_position = ramp_direction * ramp_coordinate + Eigen::Vector3f::UnitY() * width_coordinate - normal * plane_offset;
    ramp_tilt = std::atan2(-normal.x(), normal.z());
    ramp_size = Eigen::Vector2f(ramp_length, ramp_width);
    return true;
}

bool FreeSpaceDetect::planPlacementZone(const Eigen::Vector3f& trailer_pose, const Eigen::Vector2f& trailer_size, const Eigen::Vector2f& load_size)
{
    mPlacementZones.clear();
    if (!trailer_pose.allFinite() || !trailer_size.allFinite() || !load_size.allFinite() || (trailer_size.array() <= 0.0f).any() || (
            load_size.array() <= 0.0f).any())
    {
        RCLCPP_WARN(get_logger(), "Invalid trailer pose or dimensions for placement planning!");
        return false;
    }

    if ((load_size.array() > trailer_size.array()).any())
    {
        RCLCPP_WARN(get_logger(), "Load footprint does not fit within the trailer!");
        return false;
    }

    constexpr float count_tolerance = 1.0e-6f;
    const int longitudinal_count = std::floor(trailer_size.x() / load_size.x() + count_tolerance);
    const int lateral_count = std::floor(trailer_size.y() / load_size.y() + count_tolerance);
    if (longitudinal_count > std::numeric_limits<int>::max() || lateral_count > std::numeric_limits<int>::max())
    {
        RCLCPP_WARN(get_logger(), "Requested placement grid is too large!");
        return false;
    }

    const int rows = longitudinal_count;
    const int columns = lateral_count;
    const std::size_t zone_count = static_cast<std::size_t>(rows) * static_cast<std::size_t>(columns);
    if (zone_count > mPlacementZones.max_size())
    {
        RCLCPP_WARN(get_logger(), "Requested placement grid is too large!");
        return false;
    }
    mPlacementZones.reserve(zone_count);

    const float first_x = trailer_pose.x() - trailer_size.x() + 0.5f * load_size.x();
    const float first_y = trailer_pose.y() - 0.5f * static_cast<float>(columns) * load_size.y() + 0.5f * load_size.y();
    for (int row = 0; row < rows; ++row)
    {
        const float center_x = first_x + static_cast<float>(row) * load_size.x();
        for (int column = 0; column < columns; ++column)
        {
            const float center_y = first_y + static_cast<float>(column) * load_size.y();
            mPlacementZones.emplace_back(center_x, center_y);
        }
    }

    RCLCPP_INFO(get_logger(), "Planned %zu placement zones (%d rows, %d columns)!", mPlacementZones.size(), rows, columns);

    return true;
}

static bool isSweepCollisionFree(const RawCloud& cloud, const Eigen::Vector3f& from, const Eigen::Vector3f& to,
                                 const Eigen::Vector3f& load_size, const float clearance)
{
    const float half_x = 0.5f * load_size.x() + clearance;
    const float half_y = 0.5f * load_size.y() + clearance;

    const float min_x = std::min(from.x(), to.x()) - half_x;
    const float max_x = std::max(from.x(), to.x()) + half_x;
    const float min_y = std::min(from.y(), to.y()) - half_y;
    const float max_y = std::max(from.y(), to.y()) + half_y;

    const float min_z = std::min(from.z(), to.z()) - clearance;
    const float max_z = std::max(from.z(), to.z()) + load_size.z() + clearance;

    return !std::ranges::any_of(cloud, [&](const auto& point)
    {
        return pcl::isFinite(point) && point.x >= min_x && point.x <= max_x &&
               point.y >= min_y && point.y <= max_y &&
               point.z >= min_z && point.z <= max_z;
    });
}

static bool pushToTopLeft(const RawCloud& obstacle_cloud, const Eigen::Vector3f& load_size, Eigen::Vector3f& position,
                          const float min_center_x, const float min_center_y)
{
    constexpr float clearance = 0.0f;
    constexpr float initial_step = 0.02f;
    constexpr std::size_t max_iterations = 20000;

    if (!isSweepCollisionFree(obstacle_cloud, position, position, load_size, clearance))
    {
        return false;
    }

    float step = initial_step;
    bool x_first = true;

    const auto try_move = [&](const int axis, const float delta) -> bool
    {
        Eigen::Vector3f candidate = position;
        candidate[axis] += delta;

        // Keep the box inside the search boundary.
        if (axis == 0 && candidate.x() < min_center_x)
        {
            return false;
        }

        if (axis == 1 && candidate.y() < min_center_y)
        {
            return false;
        }

        if (!isSweepCollisionFree(obstacle_cloud, position, candidate,
                                  load_size, clearance))
        {
            return false;
        }

        position = candidate;
        return true;
    };

    for (std::size_t iteration = 0; iteration < max_iterations; ++iteration)
    {
        constexpr float min_step = 0.001f;
        bool moved_x = false;
        bool moved_y = false;

        // Alternate the movement priority.
        if (x_first)
        {
            moved_x = try_move(0, -step);
            moved_y = try_move(1, -step);
        }
        else
        {
            moved_y = try_move(1, -step);
            moved_x = try_move(0, -step);
        }

        x_first = !x_first;

        // At least one direction can still move.
        if (moved_x || moved_y)
        {
            continue;
        }

        // Both directions are blocked.
        if (step <= min_step)
        {


            return true;
        }

        // Reduce the step size and retry both directions.
        step = std::max(step * 0.5f, min_step);
    }

    return false;
}

bool FreeSpaceDetect::locateAvailableArea(const RawCloud::Ptr& trailer_cloud, const Eigen::Vector3f& load_size, Eigen::Vector3f& slot_position) const
{
    if (!trailer_cloud || trailer_cloud->size() < 100 || !load_size.allFinite() || (load_size.array() <= 0.0f).any())
    {
        RCLCPP_WARN(get_logger(), "Invalid point cloud or load dimensions for available-area detection!");
        return false;
    }

    const auto floor_cloud = std::make_shared<RawCloud>();
    const auto cloud_without_floor = std::make_shared<RawCloud>();
    const float floor_z = feature3d::detectFloor(*trailer_cloud, floor_cloud, cloud_without_floor);
    if (floor_cloud->size() < 100)
    {
        RCLCPP_WARN(get_logger(), "Failed to detect the trailer floor!");
        return false;
    }

    constexpr float resolution = 0.1f;
    OrthographicProjector<pcl::PointXYZ> projector(View::TOP, resolution);
    projector.setCloud(floor_cloud);
    const cv::Mat projection = projector.projection();
    cv::Mat floor_img;
    filter2d::close(projection, floor_img, 1);

    cv::Mat debug_img;
    cv::cvtColor(floor_img, debug_img, cv::COLOR_GRAY2BGR);

    int footprint_width = static_cast<int>(std::ceil(load_size.x() / resolution));
    int footprint_height = static_cast<int>(std::ceil(load_size.y() / resolution));
    footprint_width += (footprint_width % 2 == 0);
    footprint_height += (footprint_height % 2 == 0);
    if (footprint_width > floor_img.cols || footprint_height > floor_img.rows)
    {
        RCLCPP_WARN(get_logger(), "Load footprint is too large for the floor projection!");
        return false;
    }

    const cv::Mat footprint = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(footprint_width, footprint_height));
    cv::Mat available_img;
    cv::erode(floor_img, available_img, footprint, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));

    std::vector<cv::Point> feasible_points;
    cv::findNonZero(available_img, feasible_points);
    if (feasible_points.empty())
    {
        publishDebugImg(debug_img);
        RCLCPP_WARN(get_logger(), "Can't find any available area on the image!");
        return false;
    }

    std::ranges::sort(feasible_points, std::less{}, &cv::Point::x);
    const auto slot_position_img = std::ranges::max_element(feasible_points.begin(),
                                                            feasible_points.begin() +
                                                            std::min<long>(60, static_cast<long>(feasible_points.size())),
                                                            {}, &cv::Point::y);
    const auto slot_coordinate_nominal = projector.getCoordinate(*slot_position_img);
    const auto nearest_zone = std::ranges::min_element(mPlacementZones, {}, [slot_coordinate_nominal](const Eigen::Vector2f& zone)
                                                           {
                                                               return (zone - slot_coordinate_nominal).squaredNorm();
                                                           });
    slot_position = Eigen::Vector3f(nearest_zone->x(), nearest_zone->y(), floor_z + 0.01f);

    const cv::Rect slot_rect(slot_position_img->x - footprint_width / 2, slot_position_img->y - footprint_height / 2,
                             footprint_width, footprint_height);
    cv::rectangle(debug_img, slot_rect, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::circle(debug_img, *slot_position_img, 2, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    publishDebugImg(debug_img);

    if (constexpr uint32_t points_tolerance = 20;
        filter3d::checkSpace(*cloud_without_floor, slot_position, load_size) < points_tolerance)
    {
        return true;
    }

    const auto slot_cloud = std::make_shared<RawCloud>();
    constexpr float dilate_size = 0.1f;
    ROI slot_roi = {.min_x = -0.5f * load_size.x() - dilate_size, .max_x = 0.5f * load_size.x() + dilate_size,
                    .min_y = -0.5f * load_size.y() - dilate_size, .max_y = 0.5f * load_size.y() + dilate_size, .min_z = 0.0,
                    .max_z = load_size.z()};
    filter3d::getCloud(*cloud_without_floor, slot_cloud, nullptr, slot_roi, slot_position);

    if (Eigen::Vector3f refined_slot_position = slot_position + Eigen::Vector3f(0.5f * load_size.x(), 0.5f * load_size.y(), 0.0f);
        !pushToTopLeft(*slot_cloud, load_size, refined_slot_position, slot_position.x(), slot_position.y()))
    {
        return false;
    }
    else
    {
        slot_position = refined_slot_position;
    }

    return true;
}