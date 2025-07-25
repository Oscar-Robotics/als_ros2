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

#include <als_ros/SM.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    als_ros::MCL mcl;
    double localizationHz = mcl.getLocalizationHz();
    rclcpp::Rate loopRate(localizationHz);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(mcl.get_node_base_interface());
        //        mcl.updateParticlesByMotionModel();
        mcl.setCanUpdateScan(false);
        //        mcl.scanMatching();
        mcl.scanMatching2();
        //        mcl.calculateLikelihoodsByMeasurementModel();
        //        mcl.calculateLikelihoodsByDecisionModel();
        //        mcl.calculateGLSampledPosesLikelihood();
        //        mcl.calculateAMCLRandomParticlesRate();
        //        mcl.calculateEffectiveSampleSize();
        //        mcl.estimatePose();
        //        mcl.resampleParticles();
        //        mcl.plotScan();
        mcl.plotWorld(10.0);
        //        mcl.publishROSMessages();
        mcl.broadcastTF();
        //        mcl.plotLikelihoodMap();
        mcl.setCanUpdateScan(true);
        //        mcl.printResult();
        loopRate.sleep();
    }

    return 0;
}
