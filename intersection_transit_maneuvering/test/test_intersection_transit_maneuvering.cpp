/*
 * Copyright (C) 2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <gmock/gmock.h>
#include <ros/ros.h>
#include <cav_msgs/Maneuver.h>
#include <intersection_transit_maneuvering.h>
#include <chrono>
#include <ctime>
#include <cav_srvs/PlanTrajectory.h>
#include <math.h>
#include <string>
#include <algorithm>
#include <memory>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <ros/console.h>
#include "call_test.h"

namespace intersection_transit_maneuvering
{

/*Test Callback Operation*/
TEST(Intersection_Transit_Maneuvering_Test, Planning_Callback_Test)
{

    std::shared_ptr<call_test::CallTest> object = std::make_shared<call_test::CallTest>();
    std::shared_ptr<CallInterface> obj = object;
    
    IntersectionTransitManeuvering itm_node([&](auto msg) {}, obj);

    cav_msgs::Maneuver man0, man1;
    std::vector<cav_msgs::TrajectoryPlanPoint> points; 

    /*Create and populate the PlanTrajectory Service*/
    cav_msgs::TrajectoryPlanPoint p1, p2, p3;
    p1.controller_plugin_name = "Point1";
    p1.x = 0.0;
    p1.y = 1.0;
    p1.lane_id = "abcd";

    p2.controller_plugin_name = "Point2";
    p2.x = 0.0;
    p2.y = 2.0;
    p2.lane_id = "efgh";
    
    p3.controller_plugin_name = "Point3";
    p3.x = 0.0;
    p3.y = 3.0;
    p3.lane_id = "ijkl";


    cav_srvs::PlanTrajectoryRequest req;
    req.vehicle_state.X_pos_global = 1.5;
    req.vehicle_state.Y_pos_global = 5;
    req.vehicle_state.orientation = 0;
    req.vehicle_state.longitudinal_vel = 0.0;

    req.initial_trajectory_plan.trajectory_id = "TEST";
    req.initial_trajectory_plan.trajectory_points.push_back(p1);
    req.initial_trajectory_plan.trajectory_points.push_back(p2);
    req.initial_trajectory_plan.trajectory_points.push_back(p3); 
    cav_srvs::PlanTrajectoryResponse resp;

    //Invalid Maneuver Type
    man0.type = cav_msgs::Maneuver::LANE_FOLLOWING;
    man0.lane_following_maneuver.start_dist = 0.0;
    man0.lane_following_maneuver.end_dist = 5.0;
    man0.lane_following_maneuver.start_time = ros::Time(0.0);
    man0.lane_following_maneuver.end_time = ros::Time(1.7701);
    man0.lane_following_maneuver.lane_ids = { "1200" };

    req.maneuver_plan.maneuvers.push_back(man0);

    /*Test that the operation will throw an invalid argrument error statement due to no maneuvers being available for conversion*/
    EXPECT_THROW(itm_node.plan_trajectory_cb(req, resp), std::invalid_argument);
    ASSERT_EQ(0, resp.maneuver_status.size());


    /*Valid Maneuver Type */
    man1.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
    man1.intersection_transit_straight_maneuver.start_dist = 0.0;
    man1.intersection_transit_straight_maneuver.end_dist = 5.0;
    man1.intersection_transit_straight_maneuver.start_time = ros::Time(0.0);
    man1.intersection_transit_straight_maneuver.end_time = ros::Time(1.7701);
    man1.intersection_transit_straight_maneuver.starting_lane_id = "1200";

    req.maneuver_plan.maneuvers.pop_back();
    ASSERT_EQ(0,req.maneuver_plan.maneuvers.size());

    req.maneuver_plan.maneuvers.push_back(man1);
    
    auto plan = itm_node.plan_trajectory_cb(req, resp);

    /*Get req and resp values from the test call() function*/
    auto test_req = object->getRequest();
    auto test_resp = object->getResponse();

    /*Test call() function*/
    ASSERT_EQ(1, test_req.maneuver_plan.maneuvers.size());
    ASSERT_EQ(req.vehicle_state.X_pos_global, test_req.vehicle_state.X_pos_global);
    ASSERT_EQ(req.vehicle_state.Y_pos_global, test_req.vehicle_state.Y_pos_global);
    ASSERT_EQ(req.vehicle_state.longitudinal_vel, test_req.vehicle_state.longitudinal_vel);
    ASSERT_EQ(req.vehicle_state.orientation, test_req.vehicle_state.orientation);
    ASSERT_EQ(req.initial_trajectory_plan.trajectory_id, test_req.initial_trajectory_plan.trajectory_id);

    for(size_t i = 0; i < req.initial_trajectory_plan.trajectory_points.size(); i++)
    {
        ASSERT_EQ(req.initial_trajectory_plan.trajectory_points[i].controller_plugin_name, test_req.initial_trajectory_plan.trajectory_points[i].controller_plugin_name);
        ASSERT_EQ(req.initial_trajectory_plan.trajectory_points[i].x, test_req.initial_trajectory_plan.trajectory_points[i].x);
        ASSERT_EQ(req.initial_trajectory_plan.trajectory_points[i].y, test_req.initial_trajectory_plan.trajectory_points[i].y);
        ASSERT_EQ(req.initial_trajectory_plan.trajectory_points[i].lane_id, test_req.initial_trajectory_plan.trajectory_points[i].lane_id);
    }

    /*Assert that the maneuver status will be updated after the callback function*/
    ASSERT_EQ(0, test_resp.maneuver_status.size());
    ASSERT_EQ(1, resp.maneuver_status.size());
    ASSERT_EQ(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS, resp.maneuver_status.back());

}//End Test Case

TEST(Intersection_Transit_Maneuvering_Test, Convert_Maneuvers_Test)
{
    std::shared_ptr<CallInterface> obj;
    IntersectionTransitManeuvering itm_node([&](auto msg) {}, obj);

    std::vector<cav_msgs::Maneuver> maneuvers;

    EXPECT_THROW(itm_node.convert_maneuver_plan(maneuvers), std::invalid_argument); //Test that the operation will throw an exception due to there being no maneuvers to convert

    cav_msgs::Maneuver man0, man1, man2, man3;

    /*Invalid Maneuver Type*/
    man0.type = cav_msgs::Maneuver::LANE_FOLLOWING;
    man0.lane_following_maneuver.start_dist = 0.0;
    man0.lane_following_maneuver.end_dist = 5.0;
    man0.lane_following_maneuver.start_time = ros::Time(0.0);
    man0.lane_following_maneuver.end_time = ros::Time(1.7701);
    man0.lane_following_maneuver.lane_ids = { "1200" };

    /*Test that the operation will throw an invalid argrument error statement due to a non-applicable maneuver type being used*/
    maneuvers.push_back(man0);
    EXPECT_THROW(itm_node.convert_maneuver_plan(maneuvers), std::invalid_argument); 
    maneuvers.pop_back();

    /*Valid Maneuver Types */
    man1.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
    man1.intersection_transit_straight_maneuver.start_dist = 0.0;
    man1.intersection_transit_straight_maneuver.end_dist = 5.0;
    man1.intersection_transit_straight_maneuver.start_time = ros::Time(0.0);
    man1.intersection_transit_straight_maneuver.end_time = ros::Time(1.7701);
    man1.intersection_transit_straight_maneuver.start_speed = 25.0;
    man1.intersection_transit_straight_maneuver.end_speed = 1.0;
    man1.intersection_transit_straight_maneuver.starting_lane_id = "1200";
    man1.intersection_transit_straight_maneuver.parameters.maneuver_id = "asdf";


    man2.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN;
    man2.intersection_transit_left_turn_maneuver.start_dist = 0.0;
    man2.intersection_transit_left_turn_maneuver.end_dist = 5.0;
    man2.intersection_transit_left_turn_maneuver.start_time = ros::Time(0.0);
    man2.intersection_transit_left_turn_maneuver.end_time = ros::Time(1.7701);
    man2.intersection_transit_left_turn_maneuver.start_speed = 25.0;
    man2.intersection_transit_left_turn_maneuver.end_speed = 1.0;
    man2.intersection_transit_left_turn_maneuver.starting_lane_id = "1200";
    man2.intersection_transit_left_turn_maneuver.parameters.maneuver_id = "asdf";

    man3.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN;
    man3.intersection_transit_right_turn_maneuver.start_dist = 0.0;
    man3.intersection_transit_right_turn_maneuver.end_dist = 5.0;
    man3.intersection_transit_right_turn_maneuver.start_time = ros::Time(0.0);
    man3.intersection_transit_right_turn_maneuver.end_time = ros::Time(1.7701);
    man3.intersection_transit_right_turn_maneuver.start_speed = 25.0;
    man3.intersection_transit_right_turn_maneuver.end_speed = 1.0;
    man3.intersection_transit_right_turn_maneuver.parameters.maneuver_id = "asdf";

    man3.intersection_transit_right_turn_maneuver.starting_lane_id = "1200";


    maneuvers.push_back(man1);
    maneuvers.push_back(man2);
    maneuvers.push_back(man3);
    auto converted = itm_node.convert_maneuver_plan(maneuvers);

    /*Test Converted Values*/
    for(size_t i = 0; i < converted.size(); i++ )
    {
        ASSERT_EQ(true, converted[i].type == cav_msgs::Maneuver::LANE_FOLLOWING); //Test that each maneuver has been converted to LANE_FOLLOWING
        ASSERT_EQ("asdf", converted[i].lane_following_maneuver.parameters.maneuver_id);
        ASSERT_EQ(cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN, converted[i].lane_following_maneuver.parameters.presence_vector);
        ASSERT_EQ(0.0, converted[i].lane_following_maneuver.start_dist);
        ASSERT_EQ(5.0, converted[i].lane_following_maneuver.end_dist);
        ASSERT_EQ(ros::Time(0.0), converted[i].lane_following_maneuver.start_time);
        ASSERT_EQ(ros::Time(1.7701), converted[i].lane_following_maneuver.end_time);
        ASSERT_EQ(25.0, converted[i].lane_following_maneuver.start_speed);
        ASSERT_EQ(1.0, converted[i].lane_following_maneuver.end_speed);
    }
}




}