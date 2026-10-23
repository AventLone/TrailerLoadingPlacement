#pragma once
#include <opencv2/core/types.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>   // Core normal estimation header
#include <pcl/search/kdtree.h>

#include "../types/common.hpp"

namespace filter3d
{
void getCloud(const SemanticCloud& semantic_cloud, int label, pcl::PointCloud<pcl::PointXYZ>& target_cloud);

void getCloud(const SemanticCloud& semantic_cloud, pcl::PointCloud<pcl::PointXYZRGB>& colored_cloud);

template<class PointT>
void getCloud(const pcl::PointCloud<PointT>& src_cloud, typename pcl::PointCloud<PointT>::Ptr inliers,
              typename pcl::PointCloud<PointT>::Ptr outliers, const ROI& roi,
              const Eigen::Vector3f& base_position = Eigen::Vector3f::Zero())
{
    const ROI roi_rebased{
        roi.min_x + base_position[0], roi.max_x + base_position[0],
        roi.min_y + base_position[1], roi.max_y + base_position[1],
        roi.min_z + base_position[2], roi.max_z + base_position[2]
    };

    const size_t src_size = src_cloud.size();

    if (inliers != nullptr)
    {
        inliers->reserve(src_size);
        if (outliers != nullptr)
        {
            outliers->reserve(src_size);
            for (const PointT& point : src_cloud)
            {
                if (roi_rebased.enclose(point))
                {
                    inliers->push_back(point);
                }
                else
                {
                    outliers->push_back(point);
                }
            }
        }
        else
        {
            for (const PointT& point : src_cloud)
            {
                if (roi_rebased.enclose(point))
                {
                    inliers->push_back(point);
                }
            }
        }
    }
    else if (outliers != nullptr)
    {
        outliers->reserve(src_size);
        for (const PointT& point : src_cloud)
        {
            if (!roi_rebased.enclose(point))
            {
                outliers->push_back(point);
            }
        }
    }
}

template<class PointT>
void getCloud(const pcl::PointCloud<PointT>& src_cloud, typename pcl::PointCloud<PointT>::Ptr inliers,
              typename pcl::PointCloud<PointT>::Ptr outliers, const Eigen::Isometry3f& base_pose, const ROI& roi)
{
    const Eigen::Isometry3f base_pose_inv = base_pose.inverse();
    pcl::PointCloud<PointT> cloud_rebased;
    pcl::transformPointCloud(src_cloud, cloud_rebased, base_pose_inv);
    const size_t src_size = src_cloud.size();
    if (inliers != nullptr)
    {
        pcl::PointCloud<PointT> inliers_temp;
        inliers_temp.reserve(src_size);
        if (outliers != nullptr)
        {
            pcl::PointCloud<PointT> outliers_temp;
            outliers_temp.reserve(src_size);
            for (const PointT& point : cloud_rebased)
            {
                if (roi.enclose(point))
                {
                    inliers_temp.push_back(point);
                }
                else
                {
                    outliers_temp.push_back(point);
                }
            }
            pcl::transformPointCloud(inliers_temp, *inliers, base_pose);
            pcl::transformPointCloud(outliers_temp, *outliers, base_pose);
        }
        else
        {
            for (const PointT& point : cloud_rebased)
            {
                if (roi.enclose(point))
                {
                    inliers_temp.push_back(point);
                }
            }
            pcl::transformPointCloud(inliers_temp, *inliers, base_pose);
        }
    }
    else if (outliers != nullptr)
    {
        pcl::PointCloud<PointT> outliers_temp;
        outliers_temp.reserve(src_size);
        for (const PointT& point : cloud_rebased)
        {
            if (!roi.enclose(point))
            {
                outliers_temp.push_back(point);
            }
        }
        pcl::transformPointCloud(outliers_temp, *outliers, base_pose);
    }
}

template<class PointT>
void removeGround(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out, const float threshold = 0.01f)
{
    cloud_out.reserve(cloud_in.size());
    for (const auto& point : cloud_in)
    {
        if (point.z > threshold)
        {
            cloud_out.push_back(point);
        }
    }
}

/* Check for free space */
template<class PointT>
uint32_t checkSpace(const pcl::PointCloud<PointT>& cloud, const ROI& roi)
{
    uint32_t points_count{};

    for (const auto& point : cloud)
    {
        if (roi.enclose(point))
        {
            ++points_count;
        }
    }

    return points_count;
}

template<class PointT>
uint32_t checkSpace(const pcl::PointCloud<PointT>& cloud, const Eigen::Isometry3f& pose, const ROI& bbox)
{
    pcl::PointCloud<PointT> cloud_rebased;
    pcl::transformPointCloud(cloud, cloud_rebased, pose.inverse());

    return checkSpace(cloud_rebased, bbox);
}

template<class PointT>
uint32_t checkSpace(const pcl::PointCloud<PointT>& cloud, const Eigen::Vector3f& position, const ROI& bbox)
{
    const ROI roi = {position.x() + bbox.min_x, position.x() + bbox.max_x,
                     position.y() + bbox.min_y, position.y() + bbox.min_y,
                     position.z() + bbox.min_z, position.z() + bbox.max_z};

    return checkSpace(cloud, roi);
}

template<class PointT>
uint32_t checkSpace(const pcl::PointCloud<PointT>& cloud, const Eigen::Vector3f& position, const Eigen::Vector3f& dimensions)
{
    const ROI roi = {position.x() - 0.5f * dimensions.x(), position.x() + 0.5f * dimensions.x(),
                     position.y() - 0.5f * dimensions.y(), position.y() + 0.5f * dimensions.y(),
                     position.z(), position.z() + dimensions.z()};

    return checkSpace(cloud, roi);
}

template<class PointT>
bool normalFilter(const typename pcl::PointCloud<PointT>::Ptr& src_cloud,
                  typename pcl::PointCloud<PointT>::Ptr cloud_in,
                  typename pcl::PointCloud<PointT>::Ptr cloud_out = nullptr,
                  const int direction = 0)
{
    const size_t N = src_cloud->size();
    if (N < 10)
    {
        return false;
    }

    // ===================== 2. Configure Normal Estimation =====================
    // auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::PointCloud<pcl::Normal> normals;
    pcl::NormalEstimation<PointT, pcl::Normal> ne; // Normal estimation object

    // Set input point cloud
    ne.setInputCloud(src_cloud);

    // Create a KdTree for fast neighborhood search (REQUIRED)
    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    ne.setSearchMethod(tree);

    // Key parameter: Search radius (defines the local neighborhood)
    // Too small = noisy normals; Too large = over-smoothing
    ne.setRadiusSearch(0.05); // 5cm neighborhood (adjust for your data!)

    // ===================== 3. Compute Normals =====================
    ne.compute(normals);

    constexpr float threshold = 0.66f;
    if (cloud_in != nullptr)
    {
        cloud_in->clear();
        cloud_in->reserve(N);
        if (cloud_out != nullptr)
        {
            cloud_out->clear();
            cloud_out->reserve(N);
            for (size_t i = 0; i < N; ++i)
            {
                if (normals[i].normal[direction] > threshold)
                {
                    cloud_in->push_back(src_cloud->points[i]);
                }
                else
                {
                    cloud_out->push_back(src_cloud->points[i]);
                }
            }
        }
        else
        {
            for (size_t i = 0; i < N; ++i)
            {
                if (normals[i].normal[direction] > threshold)
                {
                    cloud_in->push_back(src_cloud->points[i]);
                }
            }
        }
    }
    else if (cloud_out != nullptr)
    {
        cloud_out->clear();
        cloud_out->reserve(N);
        for (size_t i = 0; i < N; ++i)
        {
            if (normals[i].normal[direction] <= threshold)
            {
                cloud_out->push_back(src_cloud->points[i]);
            }
        }
    }

    return true;
}
}