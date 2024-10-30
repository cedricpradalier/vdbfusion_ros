// MIT License
//
// # Copyright (c) 2022 Saurabh Gupta, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "Transform.hpp"
#include "vdbfusion/VDBVolume.h"
#include "vdbfusion/VDBColoredVolume.h"

namespace vdbfusion {
class VDBVolumeFunctions {
public:
    VDBVolumeFunctions(const std::string & ns=std::string(""), bool no_tf=false);
    void Integrate(const pcl::PointCloud<pcl::PointXYZ> & pcd, const Eigen::Vector3d & origin);
    void Integrate(const pcl::PointCloud<pcl::PointXYZRGB> & pcd, const Eigen::Vector3d & origin);
    void IntegrateColor(const sensor_msgs::PointCloud2& pcd);
    void IntegrateGeometry(const sensor_msgs::PointCloud2& pcd);
    void Clear();

    bool saveVDBVolumeGeometry(const std::string & path, bool mesh_only=false);
    bool saveVDBVolumeColor(const std::string & path, bool mesh_only=false);

protected:
    ros::NodeHandle nh_;
    std::shared_ptr<Transform> tf_;
    ros::Duration timestamp_tolerance_;

    std::shared_ptr<VDBVolume> InitVDBVolume();
protected:
    std::shared_ptr<VDBVolume> vdb_volume_;

    // PointCloud Processing
    bool preprocess_;
    bool apply_pose_;
    bool use_header_frame_;
    bool process_color_;
    float min_range_;
    float max_range_;

    // Triangle Mesh Extraction
    bool fill_holes_;
    float min_weight_;
};
}  // namespace vdbfusion
