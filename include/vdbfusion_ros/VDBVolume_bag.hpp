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
#include <sensor_msgs/PointCloud2.h>

#include "Transform.hpp"
#include "vdbfusion/VDBVolume.h"
#include "vdbfusion/VDBColoredVolume.h"
#include "vdbfusion_ros/VDBVolume_functions.hpp"
#include "vdbfusion_ros/save_vdb_volume.h"

namespace vdbfusion {

    class VDBVolumeBag : public VDBVolumeFunctions {
        public:
            VDBVolumeBag();

            void processBagFile(const std::string & bagfile);
            void processBagFile(const std::string & bagfile, const std::string & topic);
            void processBagFile(const std::string & bagfile, const std::set<std::string> & topics);

            bool saveVDBMesh();
            bool saveVDBMesh(const std::string & vdb_path);

        protected:
            std::string pcl_topic_;
            std::string vdb_path_;
    };
}  // namespace vdbfusion
