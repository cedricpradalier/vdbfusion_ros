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

#include "vdbfusion_ros/VDBVolume_bag.hpp"

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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <tf2_msgs/TFMessage.h>
#define foreach BOOST_FOREACH


vdbfusion::VDBVolumeBag::VDBVolumeBag() : vdbfusion::VDBVolumeFunctions() {
    nh_.getParam("pcl_topic", pcl_topic_);
    nh_.getParam("vdb_path", vdb_path_);
}


void vdbfusion::VDBVolumeBag::processBagFile(const std::string & bagfile) {
    std::set<std::string> sstr;
    sstr.insert(pcl_topic_);
    processBagFile(bagfile,sstr);
}

void vdbfusion::VDBVolumeBag::processBagFile(const std::string & bagfile, const std::string & topic) {
    std::set<std::string> sstr;
    sstr.insert(topic);
    processBagFile(bagfile,sstr);
}

void vdbfusion::VDBVolumeBag::processBagFile(const std::string & bagfile, const std::set<std::string> & topics) {
    rosbag::Bag bag;
    bag.open(bagfile);  // BagMode is Read by default

    typedef std::list<sensor_msgs::PointCloud2::Ptr> PCQ;
    PCQ Q1, Q2;
    PCQ *pQ1 = &Q1, *pQ2=&Q2;

    std::vector<geometry_msgs::TransformStamped> static_tf;

    size_t counter = 0;
    ROS_INFO("Processing bag '%s'",bagfile.c_str());
    for(auto s=topics.begin();s!=topics.end();s++) {
        ROS_INFO("Topic: %s",s->c_str());
    }
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        pQ2->clear();
        if (pQ1->size()>2) {
            ROS_INFO("PQ1 contains %d element",int(pQ1->size()));
        }
        for (PCQ::iterator it=pQ1->begin();it!=pQ1->end();it++) {
            sensor_msgs::PointCloud2::Ptr pc = *it;
            if (process_color_) {
                if (!IntegrateColorCheckTF(*pc)) {
                    pQ2->push_back(pc);
                }
            } else {
                if (!IntegrateGeometryCheckTF(*pc)) {
                    pQ2->push_back(pc);
                }
            }
        }
        std::swap(pQ1,pQ2);
#if 0
        if (pQ1->size()) {
            ROS_INFO("PQ1 still contains %d element",int(pQ1->size()));
        }
#endif

        // ROS_INFO("Message on %s",m.getTopic().c_str());
        sensor_msgs::PointCloud2::Ptr pc = m.instantiate<sensor_msgs::PointCloud2>();
        if ((pc != nullptr) && ((topics.find(m.getTopic())!=topics.end()) || topics.empty())) {
#if 0
            ROS_INFO("PC in %s %f",
                    pc->header.frame_id.c_str(),
                    pc->header.stamp.toSec());
#endif
            counter += 1;
            if (counter % 100 == 0) {
                ROS_INFO("Processed %d clouds",int(counter));
            }

            for (size_t i=0;i<static_tf.size();i++) {
                static_tf[i].header.stamp = pc->header.stamp;
                tf_->setTransform(static_tf[i]);
            }

            if (process_color_) {
                if (!IntegrateColorCheckTF(*pc)) {
                    pQ1->push_back(pc);
                }
            } else {
                if (!IntegrateGeometryCheckTF(*pc)) {
                    pQ1->push_back(pc);
                }
            }
            continue;
        }
        tf2_msgs::TFMessage::ConstPtr tf = m.instantiate<tf2_msgs::TFMessage>();
        if (tf != nullptr) {
            for(size_t i=0;i<tf->transforms.size();i++) {
                if (tf->transforms[i].header.stamp == ros::Time()) {
                    static_tf.push_back(tf->transforms[i]);
                }
                tf_->setTransform(tf->transforms[i]);
#if 0
                ROS_INFO("Added tf from %s to %s at %f",
                        tf->transforms[i].header.frame_id.c_str(),
                        tf->transforms[i].child_frame_id.c_str(),
                        tf->transforms[i].header.stamp.toSec());
#endif

            }
            continue;
        }
    }

    bag.close();
    ROS_INFO("Completed bag '%s'",bagfile.c_str());
}


bool vdbfusion::VDBVolumeBag::saveVDBMesh(const std::string & path) {
    if (process_color_) {
        return saveVDBVolumeColor(path);
    } else {
        return saveVDBVolumeGeometry(path);
    }
}

bool vdbfusion::VDBVolumeBag::saveVDBMesh() {
    return saveVDBMesh(vdb_path_);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "vdbfusion_bagnode");
    vdbfusion::VDBVolumeBag vdb_volume_bag;
    for (int c=1;c<argc;c++) {
        vdb_volume_bag.processBagFile(argv[c]);
    }
    return 0;
}
