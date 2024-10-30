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


vdbfusion::VDBVolumeNode::VDBVolumeNode() : vdbfusion::VDBVolumeFunctions() {
    const int queue_size = 500;
    std::string pcl_topic;
    nh_.getParam("pcl_topic", pcl_topic);

    if (process_color_) {
        sub_ = nh_.subscribe(pcl_topic, queue_size, &vdbfusion::VDBVolumeFunctions::IntegrateColor, (vdbfusion::VDBVolumeFunctions*)this);
    } else {
        sub_ = nh_.subscribe(pcl_topic, queue_size, &vdbfusion::VDBVolumeFunctions::IntegrateGeometry, (vdbfusion::VDBVolumeFunctions*)this);
    }
    srv_ = nh_.advertiseService("save_vdb_volume", &vdbfusion::VDBVolumeNode::saveVDBVolume, this);

    ROS_INFO("Use '/save_vdb_volume' service to save the integrated volume");
}


bool vdbfusion::VDBVolumeNode::saveVDBVolume(vdbfusion_ros::save_vdb_volume::Request& path,
                                             vdbfusion_ros::save_vdb_volume::Response& response) {
    if (process_color_) {
        return saveVDBVolumeColor(path.path);
    } else {
        return saveVDBVolumeGeometry(path.path);
    }
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "vdbfusion_rosnode");
    vdbfusion::VDBVolumeNode vdb_volume_node;
    ros::spin();
}
