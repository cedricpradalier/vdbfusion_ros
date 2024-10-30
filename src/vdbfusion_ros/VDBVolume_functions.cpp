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

#include "vdbfusion_ros/VDBVolume_ros.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Core>
#include <vector>

#include "happly.h"
#include "igl/write_triangle_mesh.h"
#include "openvdb/openvdb.h"

namespace {
typedef std::tuple<std::vector<openvdb::Vec3i>, std::vector<Eigen::Vector3d>> ColorsAndPoints;

ColorsAndPoints PointcloudToColorsAndPoints(const pcl::PointCloud<pcl::PointXYZRGB>& pcl) {
    std::vector<openvdb::Vec3i> colors;
    std::vector<Eigen::Vector3d> points;
    colors.reserve(pcl.size());
    points.reserve(pcl.size());
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto& point) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            colors.emplace_back(openvdb::Vec3i(point.r, point.g, point.b));
            points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
    });
    return std::forward_as_tuple(std::move(colors), std::move(points));
}

std::vector<Eigen::Vector3d> PointcloudToPoints(const pcl::PointCloud<pcl::PointXYZ>& pcl) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(pcl.size());
    std::for_each(pcl.points.begin(), pcl.points.end(), [&](const auto& point) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
        }
    });
    return points;
}

template <class PointT>
    void PreProcessCloud(pcl::PointCloud<PointT>& points, float min_range, float max_range) {
        points.erase(std::remove_if(points.begin(), points.end(),
                    [&](auto p) { return p.getVector3fMap().norm() > max_range; }),
                points.end());
        points.erase(std::remove_if(points.begin(), points.end(),
                    [&](auto p) { return p.getVector3fMap().norm() < min_range; }),
                points.end());
    }

}  // namespace

std::shared_ptr<vdbfusion::VDBVolume> vdbfusion::VDBVolumeFunctions::InitVDBVolume() {
    float voxel_size;
    float sdf_trunc;
    bool space_carving;
    bool process_color;

    nh_.getParam("voxel_size", voxel_size);
    nh_.getParam("sdf_trunc", sdf_trunc);
    nh_.getParam("space_carving", space_carving);
    nh_.getParam("process_color", process_color);

    std::shared_ptr<VDBVolume> vdb_volume;
    if (process_color) {
       vdb_volume.reset(new VDBColoredVolume(voxel_size, sdf_trunc, space_carving));
    } else {
        vdb_volume.reset(new VDBVolume(voxel_size, sdf_trunc, space_carving));
    }

    return vdb_volume;
}

vdbfusion::VDBVolumeFunctions::VDBVolumeFunctions(const std::string & ns, bool no_tf) : nh_(ns),vdb_volume_(InitVDBVolume()) {
    openvdb::initialize();

    nh_.getParam("preprocess", preprocess_);
    nh_.getParam("apply_pose", apply_pose_);
    nh_.getParam("use_header_frame", use_header_frame_);
    nh_.getParam("process_color", process_color_);
    nh_.getParam("min_range", min_range_);
    nh_.getParam("max_range", max_range_);

    nh_.getParam("fill_holes", fill_holes_);
    nh_.getParam("min_weight", min_weight_);

    int32_t tol;
    nh_.getParam("timestamp_tolerance_ns", tol);
    timestamp_tolerance_ = ros::Duration(0, tol);

    if (!no_tf) {
        tf_.reset(new vdbfusion::Transform(nh_));
    }


}

void vdbfusion::VDBVolumeFunctions::Clear() {
    vdb_volume_ = InitVDBVolume();
}

void vdbfusion::VDBVolumeFunctions::Integrate(const pcl::PointCloud<pcl::PointXYZRGB> & pcd, const Eigen::Vector3d & origin) {
    if (!pcd.size()) {
        return;
    }

    auto [colors, points] = PointcloudToColorsAndPoints(pcd);
    std::shared_ptr<VDBColoredVolume> colored_volume = std::dynamic_pointer_cast<VDBColoredVolume>(vdb_volume_);
    assert(colored_volume);
    colored_volume->Integrate(points, colors, origin, [](float /*unused*/) { return 1.0; });
}

void vdbfusion::VDBVolumeFunctions::Integrate(const pcl::PointCloud<pcl::PointXYZ> & pcd, const Eigen::Vector3d & origin) {
    if (!pcd.size()) {
        return;
    }

    auto points = PointcloudToPoints(pcd);

    vdb_volume_->Integrate(points, origin, [](float /*unused*/) { return 1.0; });
}

bool vdbfusion::VDBVolumeFunctions::IntegrateColorCheckTF(const sensor_msgs::PointCloud2& pcd) {
	geometry_msgs::TransformStamped transform;
	sensor_msgs::PointCloud2 pcd_ = pcd;

	pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	pcl::moveFromROSMsg(pcd_, point_cloud);
	if (preprocess_) {
		PreProcessCloud(point_cloud, min_range_, max_range_);
	}

	if (!point_cloud.size()) {
		return true;
	}

	bool tf_available = false;
	if (use_header_frame_) {
		tf_available = tf_->lookUpTransform(pcd.header.frame_id, pcd.header.stamp, timestamp_tolerance_, transform);
	} else {
		tf_available = tf_->lookUpTransform(pcd.header.stamp, timestamp_tolerance_, transform);
	}
	if (!tf_available) {
		return false;
	}
	// ROS_INFO("Transform available");
	if (apply_pose_) {
		pcl_ros::transformPointCloud(point_cloud, point_cloud, transform.transform);
	}

	auto [colors, points] = PointcloudToColorsAndPoints(point_cloud);

	const auto& x = transform.transform.translation.x;
	const auto& y = transform.transform.translation.y;
	const auto& z = transform.transform.translation.z;
	auto origin = Eigen::Vector3d(x, y, z);
	// TODO Add flag to decide if we need color or not
	std::shared_ptr<VDBColoredVolume> colored_volume = std::dynamic_pointer_cast<VDBColoredVolume>(vdb_volume_);
	assert(colored_volume);
	colored_volume->Integrate(points, colors, origin, [](float /*unused*/) { return 1.0; });
	return true;
}
bool vdbfusion::VDBVolumeFunctions::IntegrateGeometryCheckTF(const sensor_msgs::PointCloud2& pcd) {
	geometry_msgs::TransformStamped transform;
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	sensor_msgs::PointCloud2 pcd_ = pcd;
	pcl::moveFromROSMsg(pcd_, point_cloud);
	if (preprocess_) {
		PreProcessCloud(point_cloud, min_range_, max_range_);
	}

	if (!point_cloud.size()) {
		return true;
	}

	bool tf_available = false;
	if (use_header_frame_) {
		tf_available = tf_->lookUpTransform(pcd.header.frame_id, pcd.header.stamp, timestamp_tolerance_, transform);
	} else {
		tf_available = tf_->lookUpTransform(pcd.header.stamp, timestamp_tolerance_, transform);
	}
	if (!tf_available) {
		return false;
	}
	if (apply_pose_) {
		pcl_ros::transformPointCloud(point_cloud, point_cloud, transform.transform);
	}

	auto points = PointcloudToPoints(point_cloud);

	const auto& x = transform.transform.translation.x;
	const auto& y = transform.transform.translation.y;
	const auto& z = transform.transform.translation.z;
	auto origin = Eigen::Vector3d(x, y, z);
	// TODO Add flag to decide if we need color or not
	vdb_volume_->Integrate(points, origin, [](float /*unused*/) { return 1.0; });
	return true;
}

bool vdbfusion::VDBVolumeFunctions::saveVDBVolumeColor(const std::string & path, bool mesh_only) {
    ROS_INFO("Saving the colored mesh and VDB grid files ...");
    std::string volume_name = path;
    if (!mesh_only) {
        openvdb::io::File(volume_name + "_grid.vdb").write({vdb_volume_->tsdf_});
    }

    // Run marching cubes and save a .ply file
    std::shared_ptr<VDBColoredVolume> colored_volume = std::dynamic_pointer_cast<VDBColoredVolume>(vdb_volume_);
    assert(colored_volume);
    auto [vertices, triangles, colors] =
        colored_volume->ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

    // Suppose these hold your data
    std::vector<std::array<double, 3>> meshVertexPositions;
    for (size_t i = 0; i < vertices.size(); i++) {
        std::array<double, 3> vertex = {vertices[i][0], vertices[i][1], vertices[i][2]};
        meshVertexPositions.emplace_back(vertex);
    }

    std::vector<std::vector<size_t>> meshFaceIndices;
    for (size_t i = 0; i < triangles.size(); i++) {
        std::vector<size_t> triangle = {static_cast<size_t>(triangles[i][0]),
                                        static_cast<size_t>(triangles[i][1]),
                                        static_cast<size_t>(triangles[i][2])};
        meshFaceIndices.emplace_back(triangle);
    }

    std::vector<std::array<double, 3>> meshVertexColors;
    for (size_t i = 0; i < colors.size(); i++) {
        std::array<double, 3> triangle = {colors[i][0], colors[i][1], colors[i][2]};
        meshVertexColors.emplace_back(triangle);
    }

    // Create an empty object
    happly::PLYData plyOut;

    // Add mesh data (elements are created automatically)
    plyOut.addVertexPositions(meshVertexPositions);
    plyOut.addVertexColors(meshVertexColors);
    plyOut.addFaceIndices(meshFaceIndices);

    // Write the object to file
    std::string filename = volume_name + "_mesh.ply";
    plyOut.write(filename, happly::DataFormat::Binary);

    ROS_INFO("Done saving the mesh and VDB grid files in file %s", filename.c_str());
    return true;
}

bool vdbfusion::VDBVolumeFunctions::saveVDBVolumeGeometry(const std::string & path, bool mesh_only) {
    ROS_INFO("Saving the mesh and VDB grid files ...");
    std::string volume_name = path;
    if (!mesh_only) {
        openvdb::io::File(volume_name + "_grid.vdb").write({vdb_volume_->tsdf_});
    }

    // Run marching cubes and save a .ply file
    auto [vertices, triangles] =
        this->vdb_volume_->ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

    Eigen::MatrixXd V(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); i++) {
        V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
    }

    Eigen::MatrixXi F(triangles.size(), 3);
    for (size_t i = 0; i < triangles.size(); i++) {
        F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
    }
    igl::write_triangle_mesh(volume_name + "_mesh.ply", V, F, igl::FileEncoding::Binary);
    ROS_INFO("Done saving the mesh and VDB grid files");
    return true;
}

