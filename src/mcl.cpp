/****************************************************************************
 * als_ros: An Advanced Localization System for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <als_ros/MCL.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<als_ros::MCL>();
    double localizationHz = node->getLocalizationHz();
    rclcpp::Rate loopRate(localizationHz);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node->get_node_base_interface());
        node->updateParticlesByMotionModel();
        node->setCanUpdateScan(false);
        node->calculateLikelihoodsByMeasurementModel();
        node->calculateLikelihoodsByDecisionModel();
        node->calculateGLSampledPosesLikelihood();
        node->calculateAMCLRandomParticlesRate();
        node->calculateEffectiveSampleSize();
        node->estimatePose();
        node->resampleParticles();
        // mcl.plotScan();
        // mcl.plotWorld(50.0);
        node->publishROSMessages();
        node->broadcastTF();
        // mcl.plotLikelihoodMap();
        node->setCanUpdateScan(true);
        loopRate.sleep();
    }

    return 0;
}
