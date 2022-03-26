#include "VDBVolume_ros.hpp"

#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <vector>

#include "igl/write_triangle_mesh.h"
#include "openvdb/openvdb.h"
#include "ros/ros.h"
#include "type_conversions.hpp"

vdbfusion::VDBVolumeROS::VDBVolumeROS(float voxel_size,
                                      float sdf_trunc,
                                      bool space_carving /* = false*/,
                                      bool fill_holes /* = true*/,
                                      float min_weight /* = 5.0*/)
    : VDBVolume(voxel_size, sdf_trunc, space_carving),
      fill_holes_(fill_holes),
      min_weight_(min_weight) {}

void vdbfusion::VDBVolumeROS::integrate(const vdbfusion_ros::pointcloud_with_origin& pcl) {
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0);

    tf::originPointMsgToEigen(pcl.origin, origin);
    tf::pclSensorMsgToEigen(pcl.pcl, points);
    this->Integrate(points, origin, [](float /*unused*/) { return 1.0; });
}

bool vdbfusion::VDBVolumeROS::saveVolume(vdbfusion_ros::save_volume::Request& request, 
      vdbfusion_ros::save_volume::Response& response) {
    // Store the grid results to disks
    std::string map_name = request.path;
    {
        ROS_INFO("Writing VDB grid to disk");
        auto tsdf_grid = this->tsdf_;
        std::string filename = map_name + ".vdb";
        openvdb::io::File(filename).write({tsdf_grid});
    }

    // Run marching cubes and save a .ply file
    {
        auto [vertices, triangles] =
            this->ExtractTriangleMesh(this->fill_holes_, this->min_weight_);

        // TODO: Fix this!
        Eigen::MatrixXd V(vertices.size(), 3);
        for (size_t i = 0; i < vertices.size(); i++) {
            V.row(i) = Eigen::VectorXd::Map(&vertices[i][0], vertices[i].size());
        }

        // TODO: Also this
        Eigen::MatrixXi F(triangles.size(), 3);
        for (size_t i = 0; i < triangles.size(); i++) {
            F.row(i) = Eigen::VectorXi::Map(&triangles[i][0], triangles[i].size());
        }
        std::string filename = map_name + ".ply";
        igl::write_triangle_mesh(filename, V, F, igl::FileEncoding::Binary);
    }
    ROS_INFO("Done saving the mesh and VDB grid files");
    return true;
}

int main(int argc, char** argv) {
    openvdb::initialize();

    ros::init(argc, argv, "vdbfusion_ros");
    ros::NodeHandle n;

    float voxel_size;
    float sdf_trunc;
    bool space_carving;
    bool fill_holes;
    float min_weight;

    n.getParam("/voxel_size", voxel_size);
    n.getParam("/sdf_trunc", sdf_trunc);
    n.getParam("/space_carving", space_carving);
    n.getParam("/fill_holes", fill_holes);
    n.getParam("/min_weight", min_weight);

    vdbfusion::VDBVolumeROS tsdf_volume(voxel_size, sdf_trunc, space_carving, fill_holes,
                                        min_weight);

    ros::Subscriber sub =
        n.subscribe("pointcloud", 1000, &vdbfusion::VDBVolumeROS::integrate, &tsdf_volume);

    ros::ServiceServer srv =
        n.advertiseService("/save_volume", &vdbfusion::VDBVolumeROS::saveVolume, &tsdf_volume);

    ros::spin();

    return 0;
}