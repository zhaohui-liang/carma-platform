#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.


# Build the software and its dependencies

set -ex

source /opt/ros/kinetic/setup.bash
source /opt/autoware.ai/ros/install/setup.bash --extend

cd ~/carma_ws
sudo mkdir -p /opt/carma # Create install directory
sudo chown carma /opt/carma # Set owner to expose permissions for build
sudo chgrp carma /opt/carma # Set group to expose permissions for build

echo "Building CARMA"
# --packages-up-to traffic_incident_parser platoon_strategic
colcon build --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to rosbag_mock_drivers carma driver_shutdown
#--packages-up-to rosbag_mock_drivers
#--packages-up-to rosbag_mock_drivers carma driver_shutdown
echo "Build Complete"
