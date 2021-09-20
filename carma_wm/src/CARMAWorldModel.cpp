/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <ros/ros.h>
#include <tuple>
#include <algorithm>
#include <assert.h>
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Traits.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <lanelet2_core/geometry/Polygon.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <carma_wm/Geometry.h>
#include <queue>
#include <boost/math/special_functions/sign.hpp>

namespace carma_wm
{

std::pair<TrackPos, TrackPos> CARMAWorldModel::routeTrackPos(const lanelet::ConstArea& area) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }

  lanelet::ConstLineStrings3d outer_bound = area.outerBound();

  if (outer_bound.empty())
  {
    throw std::invalid_argument("Provided area outer bound is invalid as it contains no points");
  }

  TrackPos minPos(0, 0);
  TrackPos maxPos(0, 0);
  bool first = true;
  for (lanelet::ConstLineString3d sub_bound3d : outer_bound)
  {
    auto sub_bound = lanelet::utils::to2D(sub_bound3d);
    for (lanelet::ConstPoint2d point : sub_bound)
    {
      TrackPos tp = routeTrackPos(point);
      if (first)
      {
        minPos = maxPos = tp;
        first = false;
      }
      else if (tp.downtrack < minPos.downtrack)
      {
        minPos.downtrack = tp.downtrack;
        minPos.crosstrack = tp.crosstrack;
      }
      else if (tp.downtrack > maxPos.downtrack)
      {
        maxPos.downtrack = tp.downtrack;
        maxPos.crosstrack = tp.crosstrack;
      }
    }
  }
  return std::make_pair(minPos, maxPos);
}

lanelet::Id CARMAWorldModel::getTrafficLightId(uint16_t intersection_id, uint8_t signal_group_id)
{
  
  uint32_t temp = 0;
  temp |= intersection_id;
  temp = temp << 8;
  temp |= signal_group_id;

  if (traffic_light_ids_.find(temp) != traffic_light_ids_.end())
  {
    return traffic_light_ids_[temp];
  }
  else
  {
    return lanelet::InvalId;
  }

}

TrackPos CARMAWorldModel::routeTrackPos(const lanelet::ConstLanelet& lanelet) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }

  lanelet::ConstLineString2d centerline = lanelet::utils::to2D(lanelet.centerline());
  if (centerline.empty())
  {
    throw std::invalid_argument("Provided lanelet has invalid centerline containing no points");
  }
  auto front = centerline.front();
  return routeTrackPos(front);
}

TrackPos CARMAWorldModel::routeTrackPos(const lanelet::BasicPoint2d& point) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }

  // Find the nearest continuos shortest path centerline segment using fast map nearest search
  lanelet::Points3d near_points =
      shortest_path_filtered_centerline_view_->pointLayer.nearest(point, 1);  // Find the nearest points

  // Match point with linestring using fast map lookup
  auto lineString_1 =
      lanelet::utils::to2D(shortest_path_filtered_centerline_view_->lineStringLayer.findUsages(near_points[0])
                               .front());  // Only need the first index due to nature of route centerline

  if (lineString_1.size() == 0)
  {
    throw std::invalid_argument("Invalid route loaded. Shortest path does not have proper references");
  }

  // New approach
  // 1. Find nearest point
  // 2. Find linestring associated with nearest point
  // 3. If the nearest point is the first point on the linestring then we need to check the downtrack value
  // 4. -- If donwtrack is negative then iterate over route segments to find preceeding segment
  // 5. -- If downtrack is positive then we are on the correct segment
  // 6. If the nearest point is the last point on the linestring then we need to check downtrack value
  // 7. -- If downtrack is less then seg length then we are on the correct segment
  // 8. -- If downtrack is greater then seg length then iterate over route segments to find succeeding segment
  // 9. With correct segment identified compute segment downtrack distance
  // 10. Accumulate previos segment distances if needed.

  // Find best route segment
  lanelet::Id bestRouteSegId;
  TrackPos tp(0, 0);
  // Check for end cases

  auto indexes = shortest_path_distance_map_.getIndexFromId(near_points[0].id());
  size_t ls_i = indexes.first;
  size_t p_i = indexes.second;

  if (near_points[0].id() == lineString_1.front().id())
  {  // Nearest point is at the start of a line string
    // Get start point of cur segment and add 1
    auto next_point = lineString_1[1];
    TrackPos tp_next = geometry::trackPos(point, lineString_1.front().basicPoint(), next_point.basicPoint());

    if (tp_next.downtrack >= 0 || ls_i == 0)
    {
      bestRouteSegId = lineString_1.id();
      tp = tp_next;
      // If downtrack is positive then we are on the correct segment
    }
    else
    {
      // If downtrack is negative then iterate over route segments to find preceeding segment
      const size_t prev_ls_i = ls_i - 1;

      auto prev_centerline = lanelet::utils::to2D(shortest_path_centerlines_[prev_ls_i]);  // Get prev centerline
      tp = geometry::trackPos(point, prev_centerline[prev_centerline.size() - 2].basicPoint(),
                              prev_centerline[prev_centerline.size() - 1].basicPoint());
      tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(prev_ls_i, prev_centerline.size() - 2);
      bestRouteSegId = prev_centerline.id();
    }
  }
  else if (near_points[0].id() == lineString_1.back().id())
  {  // Nearest point is the end of a line string

    // Get end point of cur segment and subtract 1
    auto prev_prev_point = lineString_1[lineString_1.size() - 2];
    auto prev_point = lineString_1[lineString_1.size() - 1];

    TrackPos tp_prev = geometry::trackPos(point, prev_prev_point.basicPoint(), prev_point.basicPoint());

    double last_seg_length =
        shortest_path_distance_map_.distanceBetween(ls_i, lineString_1.size() - 2, lineString_1.size() - 1);

    if (tp_prev.downtrack < last_seg_length || ls_i == shortest_path_centerlines_.size() - 1)
    {
      // If downtrack is less then seg length then we are on the correct segment
      bestRouteSegId = lineString_1.id();
      tp = tp_prev;
      tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(ls_i, lineString_1.size() - 2);
    }
    else
    {
      // If downtrack is greater then seg length then we need to find the succeeding segment
      auto next_centerline = lanelet::utils::to2D(shortest_path_centerlines_[ls_i + 1]);  // Get prev centerline
      tp = geometry::trackPos(point, next_centerline[0].basicPoint(), next_centerline[1].basicPoint());
      bestRouteSegId = next_centerline.id();
    }
  }
  else
  {  // The nearest point is in the middle of a line string
    // Graph the two bounding points on the line string and call matchSegment using a 3 element segment
    // There is a guarantee from the earlier if statements that near_points[0] will always be located at an index within
    // the exclusive range (0,lineString_1.size() - 1) so no need for range checks

    lanelet::BasicLineString2d subSegment = lanelet::BasicLineString2d(
        { lineString_1[p_i - 1].basicPoint(), lineString_1[p_i].basicPoint(), lineString_1[p_i + 1].basicPoint() });

    tp = std::get<0>(geometry::matchSegment(point, subSegment));  // Get track pos along centerline

    tp.downtrack += shortest_path_distance_map_.distanceToPointAlongElement(ls_i, p_i - 1);

    bestRouteSegId = lineString_1.id();
  }

  // Accumulate distance
  auto bestRouteSegIndex = shortest_path_distance_map_.getIndexFromId(bestRouteSegId);
  tp.downtrack += shortest_path_distance_map_.distanceToElement(bestRouteSegIndex.first);

  return tp;
}

class LaneletDowntrackPair
{
public:
  lanelet::ConstLanelet lanelet_;
  double downtrack_ = 0;

  LaneletDowntrackPair(lanelet::ConstLanelet lanelet, double downtrack) : lanelet_(lanelet), downtrack_(downtrack)
  {
  }
  bool operator<(const LaneletDowntrackPair& pair) const
  {
    return this->downtrack_ < pair.downtrack_;
  }
  bool operator>(const LaneletDowntrackPair& pair) const
  {
    return this->downtrack_ > pair.downtrack_;
  }
};

std::vector<lanelet::ConstLanelet> CARMAWorldModel::getLaneletsBetween(double start, double end, bool shortest_path_only,
                                                                       bool bounds_inclusive) const
{
  // Check if the route was loaded yet
  if (!route_)
  {
    throw std::invalid_argument("Route has not yet been loaded");
  }
  if (start > end)
  {
    throw std::invalid_argument("Start distance is greater than end distance");
  }

  std::vector<lanelet::ConstLanelet> output;
  std::priority_queue<LaneletDowntrackPair, std::vector<LaneletDowntrackPair>, std::greater<LaneletDowntrackPair>>
      prioritized_lanelets;

  auto lanelet_map = route_->laneletMap();
  for (lanelet::ConstLanelet lanelet : lanelet_map->laneletLayer)
  {
    if (shortest_path_only && !shortest_path_view_->laneletLayer.exists(lanelet.id()))
    {
      continue;  // Continue if we are only evaluating the shortest path and this lanelet is not part of it
    }
    lanelet::ConstLineString2d centerline = lanelet::utils::to2D(lanelet.centerline());

    auto front = centerline.front();
    auto back = centerline.back();
    TrackPos min = routeTrackPos(front);
    TrackPos max = routeTrackPos(back);

    if (!bounds_inclusive) // reduce bounds slightly to avoid including exact bounds
    {
      if (std::max(min.downtrack, start + 0.00001) > std::min(max.downtrack, end - 0.00001)
        || (start == end && (min.downtrack >= start || max.downtrack <= end)))
      {  // Check for 1d intersection
        // No intersection so continue
        continue;
      }
    }
    else
    {
      if (std::max(min.downtrack, start) > std::min(max.downtrack, end)
        || (start == end && (min.downtrack > start || max.downtrack < end)))
      {  // Check for 1d intersection
        // No intersection so continue
        continue;
      }
    }
    // Intersection has occurred so add lanelet to list
    LaneletDowntrackPair pair(lanelet, min.downtrack);
    prioritized_lanelets.push(pair);
  }

  output.reserve(prioritized_lanelets.size());
  while (!prioritized_lanelets.empty())
  {
    auto pair = prioritized_lanelets.top();
    prioritized_lanelets.pop();
    output.push_back(pair.lanelet_);
  }

  if(!shortest_path_only){
    return output;
  }
  
  //Sort lanelets according to shortest path if using shortest path
  std::vector<lanelet::ConstLanelet> sorted_output;
  for(auto llt : route_->shortestPath()){
    for(size_t i=0; i < output.size();i++){
      if(llt.id() == output[i].id()){
        sorted_output.push_back(llt);
        break;
      }
    }
  }

  return sorted_output;
}

std::vector<lanelet::BasicPoint2d> CARMAWorldModel::sampleRoutePoints(double start_downtrack, double end_downtrack,
                                                                      double step_size) const
{
  std::vector<lanelet::BasicPoint2d> output;
  if (!route_)
  {
    ROS_WARN_STREAM("Route has not yet been loaded");
    return output;
  }

  double route_end = getRouteEndTrackPos().downtrack;

  if (start_downtrack < 0 || start_downtrack > route_end || end_downtrack < 0 || end_downtrack > route_end ||
      start_downtrack > end_downtrack)
  {
    ROS_WARN_STREAM("Invalid input downtracks");
    return output;
  }

  TrackPos route_pos(start_downtrack, 0);

  if (end_downtrack == start_downtrack)
  {
    output.emplace_back(*(pointFromRouteTrackPos(route_pos)));  // If a single point was provided return that point
    return output;
  }

  output.reserve(2 + (end_downtrack - start_downtrack) / step_size);
  double downtrack = start_downtrack;
  while (downtrack < end_downtrack)
  {
    route_pos.downtrack = downtrack;
    output.emplace_back(*(pointFromRouteTrackPos(route_pos)));
    downtrack += step_size;
  }

  route_pos.downtrack = end_downtrack;
  output.emplace_back(*(pointFromRouteTrackPos(route_pos)));
  return output;
}

boost::optional<lanelet::BasicPoint2d> CARMAWorldModel::pointFromRouteTrackPos(const TrackPos& route_pos) const
{
  double downtrack = route_pos.downtrack;
  double crosstrack = route_pos.crosstrack;

  if (!route_)
  {
    ROS_DEBUG_STREAM("Route has not yet been loaded");
    return boost::none;
  }

  if (downtrack < 0 || downtrack > getRouteEndTrackPos().downtrack)
  {
    ROS_DEBUG_STREAM("Tried to convert a downtrack of: " << downtrack
                                                         << " to map point, but it did not fit in route bounds of "
                                                         << getRouteEndTrackPos().downtrack);
    return boost::none;
  }

  // Use fast lookup to identify the points before and after the provided downtrack on the route
  auto indices = shortest_path_distance_map_.getElementIndexByDistance(downtrack, true); // Get the linestring matching the provided downtrack
  size_t ls_i = std::get<0>(indices);
  size_t pt_i = std::get<1>(indices);
  
  
  auto linestring = shortest_path_centerlines_[ls_i];

  if (pt_i >= linestring.size()) {
    throw std::invalid_argument("Impossible index: pt: " + std::to_string(pt_i) + " linestring: " + std::to_string(ls_i));
  }


  double ls_downtrack = shortest_path_distance_map_.distanceToElement(ls_i);
  
  double relative_downtrack = downtrack - ls_downtrack;

  size_t centerline_size = linestring.size();


  size_t prior_idx = std::min(pt_i, centerline_size - 1);

  size_t next_idx = std::min(pt_i + 1, centerline_size - 1);

  // This if block handles the edge case where the downtrack distance has landed exactly on an existing point
  if (prior_idx == next_idx)
  {  // If both indexes are the same we are on the point
    
    if (crosstrack == 0)
    {  // Crosstrack not provided so we can return the point directly
      auto prior_point = linestring[prior_idx];

      return lanelet::BasicPoint2d(prior_point.x(), prior_point.y());
    }

    lanelet::BasicPoint2d prior_point, next_point;
    double x, y;
    if (prior_idx < centerline_size - 1)
    {  // If this is not the end point compute the crosstrack based on the next point
      prior_point = linestring[prior_idx].basicPoint2d();
      next_point = linestring[prior_idx + 1].basicPoint2d(); // No need to check bounds as this class will reject routes with fewer than 2 points in a centerline
      x = prior_point.x();
      y = prior_point.y();

    }
    else
    {  // If this is the end point compute the crosstrack based on previous point
      prior_point = linestring[prior_idx - 1].basicPoint2d();
      next_point = linestring[prior_idx].basicPoint2d();
      x = next_point.x();
      y = next_point.y();

    }

    // Compute the crosstrack
    double sigma = geometry::point_to_point_yaw(prior_point, next_point);  // Angle between route segment and x-axis
    double theta = sigma + M_PI_2;  // M_PI_2 is 90 deg. Theta is the angle to the vector from the route projected point
                                    // to the target point
    double delta_x = cos(theta) * crosstrack;
    double delta_y = sin(theta) * crosstrack;

    return lanelet::BasicPoint2d(x + delta_x, y + delta_y);
  }

  double prior_downtrack = shortest_path_distance_map_.distanceToPointAlongElement(ls_i, prior_idx);
  double next_downtrack = shortest_path_distance_map_.distanceToPointAlongElement(ls_i, next_idx);

  double prior_to_next_dist = next_downtrack - prior_downtrack;
  double prior_to_target_dist = relative_downtrack - prior_downtrack;
  double interpolation_percentage = 0;
  

  if (prior_to_next_dist < 0.000001)
  {
    interpolation_percentage = 0;
  }
  else
  {
    interpolation_percentage = prior_to_target_dist / prior_to_next_dist; // Use the percentage progress between both points to compute the downtrack point
  }

  auto prior_point = linestring[prior_idx].basicPoint2d();
  auto next_point = linestring[next_idx].basicPoint2d();
  auto delta_vec = next_point - prior_point;

  double x = prior_point.x() + interpolation_percentage * delta_vec.x();
  double y = prior_point.y() + interpolation_percentage * delta_vec.y();

  if (crosstrack != 0) // If the crosstrack is not set no need to do the costly computation.
  {  
    double sigma = geometry::point_to_point_yaw(prior_point, next_point);  // Angle between route segment and x-axis
    double theta = sigma + M_PI_2;  // M_PI_2 is 90 deg. Theta is the angle to the vector from the route projected point
                                    // to the target point
    double delta_x = cos(theta) * crosstrack;
    double delta_y = sin(theta) * crosstrack;

    x += delta_x;  // Adjust x and y of target point to account for crosstrack
    y += delta_y;
    
  }

  return lanelet::BasicPoint2d(x, y);
}

lanelet::LaneletMapConstPtr CARMAWorldModel::getMap() const
{
  return std::static_pointer_cast<lanelet::LaneletMap const>(semantic_map_);  // Cast pointer to const variant
}

LaneletRouteConstPtr CARMAWorldModel::getRoute() const
{
  return std::static_pointer_cast<const lanelet::routing::Route>(route_);  // Cast pointer to const variant
}

TrackPos CARMAWorldModel::getRouteEndTrackPos() const
{
  TrackPos p(route_length_, 0);
  return p;
}

void CARMAWorldModel::setMap(lanelet::LaneletMapPtr map, size_t map_version, bool recompute_routing_graph)
{
  // If this is the first time the map has been set, then recompute the routing graph
  if (!semantic_map_) {

    ROS_INFO_STREAM("First time map is set in carma_wm. Routing graph will be recomputed reguardless of method inputs.");

    recompute_routing_graph = true;
  }

  semantic_map_ = map;
  map_version_ = map_version;
  
  // If the routing graph should be updated then recompute it
  if (recompute_routing_graph) {

    ROS_INFO_STREAM("Building routing graph");

    TrafficRulesConstPtr traffic_rules = *(getTrafficRules(lanelet::Participants::Vehicle));
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*semantic_map_, *traffic_rules);
    map_routing_graph_ = std::move(map_graph);

    ROS_INFO_STREAM("Done building routing graph");

  }
}

size_t CARMAWorldModel::getMapVersion() const 
{
  return map_version_;
}

lanelet::LaneletMapPtr CARMAWorldModel::getMutableMap() const
{
  return semantic_map_;
}

void CARMAWorldModel::setRoute(LaneletRoutePtr route)
{
  route_ = route;
  lanelet::ConstLanelets path_lanelets(route_->shortestPath().begin(), route_->shortestPath().end());
  shortest_path_view_ = lanelet::utils::createConstSubmap(path_lanelets, {});
  computeDowntrackReferenceLine();
  // NOTE: Setting the route_length_ field here will likely result in the final lanelets final point being used. Call setRouteEndPoint to use the destination point value
  route_length_ = routeTrackPos(route_->getEndPoint().basicPoint2d()).downtrack;  // Cache the route length with
                                                                                  // consideration for endpoint
}

void CARMAWorldModel::setRouteEndPoint(const lanelet::BasicPoint3d& end_point)
{
  if (!route_) {
    throw std::invalid_argument("Route endpoint set before route was available.");
  }

  lanelet::ConstPoint3d const_end_point{ lanelet::utils::getId(), end_point.x(), end_point.y(), end_point.z() };

  route_->setEndPoint(const_end_point);
  
  route_length_ = routeTrackPos(route_->getEndPoint().basicPoint2d()).downtrack;  // Cache the route length with
                                                                                  // consideration for endpoint
}

lanelet::LineString3d CARMAWorldModel::copyConstructLineString(const lanelet::ConstLineString3d& line) const
{
  std::vector<lanelet::Point3d> coppied_points;
  coppied_points.reserve(line.size());

  for (auto basic_p : line.basicLineString())
  {
    coppied_points.push_back(lanelet::Point3d(lanelet::utils::getId(), basic_p));
  }

  return lanelet::LineString3d(lanelet::utils::getId(), coppied_points);
}

void CARMAWorldModel::computeDowntrackReferenceLine()
{
  IndexedDistanceMap distance_map;

  lanelet::routing::LaneletPath shortest_path = route_->shortestPath();
  // Build shortest path routing graph
  TrafficRulesConstPtr traffic_rules = *(getTrafficRules(lanelet::Participants::Vehicle));

  lanelet::routing::RoutingGraphUPtr shortest_path_graph =
      lanelet::routing::RoutingGraph::build(*shortest_path_view_, *traffic_rules);

  std::vector<lanelet::LineString3d> lineStrings;  // List of continuos line strings representing segments of the route
                                                   // reference line

  bool first = true;
  size_t next_index = 0;
  // Iterate over each lanelet in the shortest path this loop works by looking one lanelet ahead to detect lane changes
  for (lanelet::ConstLanelet ll : shortest_path)
  {
    next_index++;
    if (first)
    {  // For the first lanelet store its centerline and length
      lineStrings.push_back(copyConstructLineString(ll.centerline()));
      first = false;
    }
    if (next_index < shortest_path.size())
    {  // Check for remaining lanelets
      auto nextLanelet = shortest_path[next_index];
      lanelet::LineString3d nextCenterline = copyConstructLineString(nextLanelet.centerline());
      size_t connectionCount = shortest_path_graph->possiblePaths(ll, (uint32_t)2, false).size();

      if (connectionCount == 1)
      {  // Get list of connected lanelets without lanechanges. On the shortest path this should only return 1 or 0
        // No lane change
        // Append distance to current centerline
        size_t offset =
            lineStrings.size() == 0 || lineStrings.back().size() == 0 ?
                0 :
                1;  // Prevent duplicate points when concatenating. Not clear if causes an issue at lane changes
        if ((nextCenterline.size() <= 1) || (nextCenterline.size() <= 2 && offset == 1))
        {
          throw std::invalid_argument("Cannot process route with lanelet containing very short centerline");
        }
        lineStrings.back().insert(lineStrings.back().end(), nextCenterline.begin() + offset, nextCenterline.end());
      }
      else if (connectionCount == 0)
      {
        // Lane change required
        // Break the point chain when a lanechange occurs
        if (lineStrings.back().size() == 0)
          continue;  // we don't have to create empty_linestring if we already have one
                     // occurs when route is changing lanes multiple times in sequence
        lanelet::LineString3d empty_linestring;
        empty_linestring.setId(lanelet::utils::getId());
        distance_map.pushBack(lanelet::utils::to2D(lineStrings.back()));
        lineStrings.push_back(empty_linestring);
      }
      else
      {
        assert(false);  // It should not be possable to reach this point. Doing so demonstrates a bug
      }
    }
  }
  // Copy values to member variables
  while (lineStrings.back().size() == 0)
    lineStrings.pop_back();  // clear empty linestrings that was never used in the end
  shortest_path_centerlines_ = lineStrings;
  shortest_path_distance_map_ = distance_map;

  // Add length of final sections
  if (shortest_path_centerlines_.size() > shortest_path_distance_map_.size())
  {
    shortest_path_distance_map_.pushBack(lanelet::utils::to2D(lineStrings.back()));  // Record length of last continuous
                                                                                     // segment
  }

  // Since our copy constructed linestrings do not contain references to lanelets they can be added to a full map
  // instead of a submap
  shortest_path_filtered_centerline_view_ = lanelet::utils::createMap(shortest_path_centerlines_);
}

LaneletRoutingGraphConstPtr CARMAWorldModel::getMapRoutingGraph() const
{
  return std::static_pointer_cast<const lanelet::routing::RoutingGraph>(map_routing_graph_);  // Cast pointer to const
                                                                                              // variant
}

lanelet::Optional<TrafficRulesConstPtr> CARMAWorldModel::getTrafficRules(const std::string& participant) const
{
  lanelet::Optional<TrafficRulesConstPtr> optional_ptr;
  // Create carma traffic rules object
  try
  {
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::traffic_rules::CarmaUSTrafficRules::Location, participant);

    auto carma_traffic_rules = std::make_shared<lanelet::traffic_rules::CarmaUSTrafficRules>();

    carma_traffic_rules = std::static_pointer_cast<lanelet::traffic_rules::CarmaUSTrafficRules>(
        lanelet::traffic_rules::TrafficRulesPtr(std::move(traffic_rules)));
    carma_traffic_rules->setConfigSpeedLimit(config_speed_limit_);

    optional_ptr = std::static_pointer_cast<const lanelet::traffic_rules::CarmaUSTrafficRules>(carma_traffic_rules);
  }
  catch (const lanelet::InvalidInputError& e)
  {
    return optional_ptr;
  }

  return optional_ptr;
}

lanelet::Optional<cav_msgs::RoadwayObstacle>
CARMAWorldModel::toRoadwayObstacle(const cav_msgs::ExternalObject& object) const
{
  if (!semantic_map_ || semantic_map_->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }

  lanelet::BasicPoint2d object_center(object.pose.pose.position.x, object.pose.pose.position.y);

  auto nearestLaneletBoost = getIntersectingLanelet(object);

  if (!nearestLaneletBoost)
    return boost::none;

  lanelet::Lanelet nearestLanelet = nearestLaneletBoost.get();

  cav_msgs::RoadwayObstacle obs;
  obs.object = object;
  obs.connected_vehicle_type.type =
      cav_msgs::ConnectedVehicleType::NOT_CONNECTED;  // TODO No clear way to determine automation state at this time
  obs.lanelet_id = nearestLanelet.id();

  carma_wm::TrackPos obj_track_pos = geometry::trackPos(nearestLanelet, object_center);
  obs.down_track = obj_track_pos.downtrack;
  obs.cross_track = obj_track_pos.crosstrack;

  for (auto prediction : object.predictions)
  {
    // Compute prediction polygon
    lanelet::BasicPolygon2d prediction_polygon =
        geometry::objectToMapPolygon(prediction.predicted_position, object.size);
    lanelet::BasicPoint2d prediction_center(prediction.predicted_position.position.x,
                                            prediction.predicted_position.position.y);

    auto predNearestLanelet = semantic_map_->laneletLayer.nearest(prediction_center, 1)[0];

    carma_wm::TrackPos pred_track_pos = geometry::trackPos(predNearestLanelet, prediction_center);

    obs.predicted_lanelet_ids.emplace_back(predNearestLanelet.id());
    obs.predicted_cross_tracks.emplace_back(pred_track_pos.crosstrack);
    obs.predicted_down_tracks.emplace_back(pred_track_pos.downtrack);

    // Since the predictions are having their lanelet ids matched based on the nearest nounding box search rather than
    // checking for intersection The id confidence will be set to 90% of the position confidence
    obs.predicted_lanelet_id_confidences.emplace_back(0.9 * prediction.predicted_position_confidence);
    obs.predicted_cross_track_confidences.emplace_back(0.9 * prediction.predicted_position_confidence);
    obs.predicted_down_track_confidences.emplace_back(0.9 * prediction.predicted_position_confidence);
  }

  return obs;
}

void CARMAWorldModel::setRoadwayObjects(const std::vector<cav_msgs::RoadwayObstacle>& rw_objs)
{
  roadway_objects_ = rw_objs;
}

std::vector<cav_msgs::RoadwayObstacle> CARMAWorldModel::getRoadwayObjects() const
{
  return roadway_objects_;
}

std::vector<cav_msgs::RoadwayObstacle> CARMAWorldModel::getInLaneObjects(const lanelet::ConstLanelet& lanelet,
                                                                         const LaneSection& section) const
{
  // Get all lanelets on current lane section
  std::vector<lanelet::ConstLanelet> lane = getLane(lanelet, section);

  // Check if any roadway object is registered
  if (roadway_objects_.size() == 0)
  {
    return std::vector<cav_msgs::RoadwayObstacle>{};
  }

  // Initialize useful variables
  std::vector<cav_msgs::RoadwayObstacle> lane_objects, roadway_objects_copy = roadway_objects_;

  /*
   * Get all in lane objects
   * For each lane, we check if each object is on it by matching lanelet_id
   * Complexity N*K, where N: num of lanelets, K: num of objects
   */
  int curr_idx;
  std::queue<int> obj_idxs_queue;

  // Create an index queue for roadway objects to quickly pop the idx if associated
  // lanelet is found. This is to reduce number of objects to check as we check new lanelets
  for (size_t i = 0; i < roadway_objects_copy.size(); i++)
  {
    obj_idxs_queue.push((int)i);
  }

  // check each lanelets
  for (auto llt : lane)
  {
    int checked_queue_items = 0, to_check = obj_idxs_queue.size();
    // check each objects
    while (checked_queue_items < to_check)
    {
      curr_idx = obj_idxs_queue.front();
      obj_idxs_queue.pop();
      checked_queue_items++;

      // Check if the object is in the lanelet
      auto curr_obj = roadway_objects_copy[curr_idx];
      if (curr_obj.lanelet_id == llt.id())
      {
        // found intersecting lanelet for this object
        lane_objects.push_back(curr_obj);
      }
      // handle a case where an object might be lane-changing, so check adjacent ids
      // a bit faster than checking intersection solely as && is left-to-right evaluation
      else if (((map_routing_graph_->left(llt) && curr_obj.lanelet_id == map_routing_graph_->left(llt).get().id()) ||
                (map_routing_graph_->right(llt) && curr_obj.lanelet_id == map_routing_graph_->right(llt).get().id())) &&
               boost::geometry::intersects(
                   llt.polygon2d().basicPolygon(),
                   geometry::objectToMapPolygon(curr_obj.object.pose.pose, curr_obj.object.size)))
      {
        // found intersecting lanelet for this object
        lane_objects.push_back(curr_obj);
      }
      else
      {
        // did not find suitable lanelet, so it will be processed again for next lanelet
        obj_idxs_queue.push(curr_idx);
      }
    }
  }

  return lane_objects;
}

lanelet::Optional<lanelet::Lanelet>
CARMAWorldModel::getIntersectingLanelet(const cav_msgs::ExternalObject& object) const
{
  // Check if the map is loaded yet
  if (!semantic_map_ || semantic_map_->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }

  lanelet::BasicPoint2d object_center(object.pose.pose.position.x, object.pose.pose.position.y);
  lanelet::BasicPolygon2d object_polygon = geometry::objectToMapPolygon(object.pose.pose, object.size);

  auto nearestLanelet = semantic_map_->laneletLayer.nearest(
      object_center, 1)[0];  // Since the map contains lanelets there should always be at least 1 element

  // Check if the object is inside or intersecting this lanelet
  // If no intersection then the object can be considered off the road and does not need to processed
  if (!boost::geometry::intersects(nearestLanelet.polygon2d().basicPolygon(), object_polygon))
  {
    return boost::none;
  }
  return nearestLanelet;
}

lanelet::Optional<double> CARMAWorldModel::distToNearestObjInLane(const lanelet::BasicPoint2d& object_center) const
{
  // Check if the map is loaded yet
  if (!semantic_map_ || semantic_map_->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }

  // return empty if there is no object nearby
  if (roadway_objects_.size() == 0)
    return boost::none;

  // Get the lanelet of this point
  auto curr_lanelet = semantic_map_->laneletLayer.nearest(object_center, 1)[0];

  // Check if this point at least is actually within this lanelet; otherwise, it wouldn't be "in-lane"
  if (!boost::geometry::within(object_center, curr_lanelet.polygon2d().basicPolygon()))
    throw std::invalid_argument("Given point is not within any lanelet");

  std::vector<cav_msgs::RoadwayObstacle> lane_objects = getInLaneObjects(curr_lanelet);

  // return empty if there is no object in the lane
  if (lane_objects.size() == 0)
    return boost::none;

  // Record the closest distance out of all polygons, 4 points each
  double min_dist = INFINITY;
  for (auto obj : roadway_objects_)
  {
    lanelet::BasicPolygon2d object_polygon = geometry::objectToMapPolygon(obj.object.pose.pose, obj.object.size);

    // Point to closest edge on polygon distance by boost library
    double curr_dist = lanelet::geometry::distance(object_center, object_polygon);
    if (min_dist > curr_dist)
      min_dist = curr_dist;
  }

  // Return the closest distance out of all polygons
  return min_dist;
}

lanelet::Optional<std::tuple<TrackPos, cav_msgs::RoadwayObstacle>>
CARMAWorldModel::getNearestObjInLane(const lanelet::BasicPoint2d& object_center, const LaneSection& section) const
{
  // Check if the map is loaded yet
  if (!semantic_map_ || semantic_map_->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }

  // return empty if there is no object nearby
  if (roadway_objects_.size() == 0)
    return boost::none;

  // Get the lanelet of this point
  auto curr_lanelet = semantic_map_->laneletLayer.nearest(object_center, 1)[0];

  // Check if this point at least is actually within this lanelet; otherwise, it wouldn't be "in-lane"
  if (!boost::geometry::within(object_center, curr_lanelet.polygon2d().basicPolygon()))
    throw std::invalid_argument("Given point is not within any lanelet");

  // Get objects that are in the lane
  std::vector<cav_msgs::RoadwayObstacle> lane_objects = getInLaneObjects(curr_lanelet, section);

  // return empty if there is no object in the lane
  if (lane_objects.size() == 0)
    return boost::none;

  // Get the lane that is including this lanelet
  std::vector<lanelet::ConstLanelet> lane_section = getLane(curr_lanelet, section);

  std::vector<double> object_downtracks, object_crosstracks;
  std::vector<int> object_idxs;
  std::queue<int> obj_idxs_queue;
  double base_downtrack = 0;
  double input_obj_downtrack = 0;
  int curr_idx = 0;

  // Create an index queue for in lane objects to quickly pop the idx if associated
  // lanelet is found. This is to reduce number of objects to check as we check new lanelets
  for (size_t i = 0; i < lane_objects.size(); i++)
  {
    obj_idxs_queue.push((int)i);
  }

  // For each lanelet, check if each object is inside it. if so, calculate downtrack
  for (auto llt : lane_section)
  {
    int checked_queue_items = 0, to_check = obj_idxs_queue.size();

    // check each remaining objects
    while (checked_queue_items < to_check)
    {
      curr_idx = obj_idxs_queue.front();
      obj_idxs_queue.pop();
      checked_queue_items++;

      // if the object is on it, store its total downtrack distance
      if (lane_objects[curr_idx].lanelet_id == llt.id())
      {
        object_downtracks.push_back(base_downtrack + lane_objects[curr_idx].down_track);
        object_crosstracks.push_back(lane_objects[curr_idx].cross_track);
        object_idxs.push_back(curr_idx);
      }
      // if it's not on it, try adjacent lanelets because the object could be lane changing
      else if ((map_routing_graph_->left(llt) &&
                lane_objects[curr_idx].lanelet_id == map_routing_graph_->left(llt).get().id()) ||
               (map_routing_graph_->right(llt) &&
                lane_objects[curr_idx].lanelet_id == map_routing_graph_->right(llt).get().id()))
      {
        // no need to check intersection as the objects are guaranteed to be intersecting this lane
        lanelet::BasicPoint2d obj_center(lane_objects[curr_idx].object.pose.pose.position.x,
                                         lane_objects[curr_idx].object.pose.pose.position.y);
        TrackPos new_tp = geometry::trackPos(llt, obj_center);
        object_downtracks.push_back(base_downtrack + new_tp.downtrack);
        object_crosstracks.push_back(lane_objects[curr_idx].cross_track);
        object_idxs.push_back(curr_idx);
      }
      else
      {
        // the object is not in the lanelet if above conditions do not meet
        obj_idxs_queue.push(curr_idx);
      }
    }
    // try to update object_center's downtrack
    if (curr_lanelet.id() == llt.id())
      input_obj_downtrack = base_downtrack + geometry::trackPos(llt, object_center).downtrack;

    // this lanelet's entire centerline as downtrack
    base_downtrack +=
        geometry::trackPos(llt.centerline().back().basicPoint2d(), llt.centerline().front().basicPoint2d(),
                           llt.centerline().back().basicPoint2d())
            .downtrack;
  }

  // compare with input's downtrack and return the min_dist
  size_t min_idx = 0;
  double min_dist = INFINITY;
  for (size_t idx = 0; idx < object_downtracks.size(); idx++)
  {
    if (min_dist > std::fabs(object_downtracks[idx] - input_obj_downtrack))
    {
      min_dist = std::fabs(object_downtracks[idx] - input_obj_downtrack);
      min_idx = idx;
    }
  }

  // if before the parallel line with the start of the llt that crosses given object_center, neg downtrack.
  // if left to the parallel line with the centerline of the llt that crosses given object_center, pos crosstrack
  return std::tuple<TrackPos, cav_msgs::RoadwayObstacle>(
      TrackPos(object_downtracks[min_idx] - input_obj_downtrack,
               object_crosstracks[min_idx] - geometry::trackPos(curr_lanelet, object_center).crosstrack),
      lane_objects[object_idxs[min_idx]]);
}

lanelet::Optional<std::tuple<TrackPos, cav_msgs::RoadwayObstacle>>
CARMAWorldModel::nearestObjectAheadInLane(const lanelet::BasicPoint2d& object_center) const
{
  return getNearestObjInLane(object_center, LANE_AHEAD);
}

lanelet::Optional<std::tuple<TrackPos, cav_msgs::RoadwayObstacle>>
CARMAWorldModel::nearestObjectBehindInLane(const lanelet::BasicPoint2d& object_center) const
{
  return getNearestObjInLane(object_center, LANE_BEHIND);
}
std::vector<lanelet::ConstLanelet> CARMAWorldModel::getLane(const lanelet::ConstLanelet& lanelet,
                                                            const LaneSection& section) const
{
  // Check if the map is loaded yet
  if (!semantic_map_ || semantic_map_->laneletLayer.size() == 0)
  {
    throw std::invalid_argument("Map is not set or does not contain lanelets");
  }

  // Check if the lanelet is in map
  if (semantic_map_->laneletLayer.find(lanelet.id()) == semantic_map_->laneletLayer.end())
  {
    throw std::invalid_argument("Lanelet is not on the map");
  }

  // Check if the lane section input is correct
  if (section != LANE_FULL && section != LANE_BEHIND && section != LANE_AHEAD)
  {
    throw std::invalid_argument("Undefined lane section is requested");
  }

  std::vector<lanelet::ConstLanelet> following_lane = { lanelet };
  std::stack<lanelet::ConstLanelet> prev_lane_helper;
  std::vector<lanelet::ConstLanelet> prev_lane;
  std::vector<lanelet::ConstLanelet> connecting_lanelet = map_routing_graph_->following(lanelet, false);

  // if only interested in following lanelets, as it is the most case
  while (connecting_lanelet.size() != 0)
  {
    following_lane.push_back(connecting_lanelet[0]);
    connecting_lanelet = map_routing_graph_->following(connecting_lanelet[0], false);
  }
  if (section == LANE_AHEAD)
    return following_lane;

  // if interested in lanelets behind
  connecting_lanelet = map_routing_graph_->previous(lanelet, false);
  while (connecting_lanelet.size() != 0)
  {
    prev_lane_helper.push(connecting_lanelet[0]);
    connecting_lanelet = map_routing_graph_->previous(connecting_lanelet[0], false);
  }

  // gather all lanelets with correct start order
  while (prev_lane_helper.size() != 0)
  {
    prev_lane.push_back(prev_lane_helper.top());
    prev_lane_helper.pop();
  }

  // if only interested in lane behind
  if (section == LANE_BEHIND)
  {
    prev_lane.push_back(lanelet);
    return prev_lane;
  }

  // if interested in full lane
  prev_lane.insert(prev_lane.end(), following_lane.begin(), following_lane.end());
  return prev_lane;
}

void CARMAWorldModel::setTrafficLightIds(uint32_t id, lanelet::Id lanelet_id)
{
  traffic_light_ids_[id] = lanelet_id;
}

void CARMAWorldModel::setConfigSpeedLimit(double config_lim)
{
  config_speed_limit_ = config_lim;
}

std::vector<lanelet::Lanelet> CARMAWorldModel::getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n)
{
  return carma_wm::query::getLaneletsFromPoint(semantic_map_, point, n);
}

std::vector<lanelet::ConstLanelet> CARMAWorldModel::getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n) const
{
  return carma_wm::query::getLaneletsFromPoint(getMap(), point, n);
}

std::vector<lanelet::Lanelet> CARMAWorldModel::nonConnectedAdjacentLeft(const lanelet::BasicPoint2d& input_point, const unsigned int n)
{
  return carma_wm::query::getLaneletsFromPoint(semantic_map_, input_point, n);
}

std::vector<lanelet::ConstLanelet> CARMAWorldModel::nonConnectedAdjacentLeft(const lanelet::BasicPoint2d& input_point, const unsigned int n) const
{
  return carma_wm::query::getLaneletsFromPoint(getMap(), input_point, n);
}

std::vector<lanelet::CarmaTrafficLightPtr> CARMAWorldModel::getLightsAlongRoute(const lanelet::BasicPoint2d& loc) const
{
  // Check if the map is loaded yet
  if (!semantic_map_ || semantic_map_->laneletLayer.empty())
  {
    ROS_ERROR_STREAM("Map is not set or does not contain lanelets");
    return {};
  }
  // Check if the route was loaded yet
  if (!route_)
  {
    ROS_ERROR_STREAM("Route has not yet been loaded");
    return {};
  }
  std::vector<lanelet::CarmaTrafficLightPtr> light_list;
  auto curr_downtrack = routeTrackPos(loc).downtrack;
  // shortpath is already sorted by distance
  for(const auto& ll : route_->shortestPath())
  {
    auto lights = semantic_map_->laneletLayer.get(ll.id()).regulatoryElementsAs<lanelet::CarmaTrafficLight>();
    if (lights.empty())
    {
      continue;
    }
    for (auto light : lights)
    {
      double light_downtrack = routeTrackPos(light->stopLine().front().front().basicPoint2d()).downtrack;
      if (light_downtrack < curr_downtrack)
      {
        continue;
      }
      light_list.push_back(light);
    }
  }
  return light_list;
}

void CARMAWorldModel::processSpatFromMsg(const cav_msgs::SPAT& spat_msg)
{
  if (!semantic_map_)
  {
    ROS_INFO_STREAM("Map is not set yet.");
    return;
  }

  if (spat_msg.intersection_state_list.empty())
  {
    ROS_WARN_STREAM("No intersection_state_list in the newly received SPAT msg. Returning...");
    return;
  }

  for (const auto& curr_intersection : spat_msg.intersection_state_list)
  {
    for (const auto& current_movement_state : curr_intersection.movement_list) 
    {
      lanelet::Id curr_light_id = getTrafficLightId(curr_intersection.id.id, current_movement_state.signal_group);

      if (curr_light_id == lanelet::InvalId)
      {
        continue;
      }
      
      auto general_regem = semantic_map_->regulatoryElementLayer.get(curr_light_id);
      
      auto lanelets_general = semantic_map_->laneletLayer.findUsages(general_regem);
      if (lanelets_general.empty())
      {
        ROS_WARN_STREAM("Received a SPAT message for traffic light that is not owned by any lanelet with intersection_id: " << (int)curr_intersection.id.id << 
                           ", and signal_group_id: " << (int)current_movement_state.signal_group);
        continue;
      }
      auto curr_light_list = lanelets_general[0].regulatoryElementsAs<lanelet::CarmaTrafficLight>();
      
      if (curr_light_list.empty())
      {
        ROS_WARN_STREAM("There was an error querying traffic light with intersection_id: " << (int)curr_intersection.id.id << 
                           ", and signal_group_id: " << (int)current_movement_state.signal_group);
        continue;
      }
      lanelet::CarmaTrafficLightPtr curr_light = curr_light_list[0];
      
      // reset states if the intersection's geometry changed 
      if (curr_light->revision_ != curr_intersection.revision)
      {
        ROS_INFO_STREAM("Received a new intersection geometry. intersection_id: " << (int)curr_intersection.id.id << 
                           ", and signal_group_id: " << (int)current_movement_state.signal_group);
        traffic_light_states_[curr_intersection.id.id].clear();
      }

      // if full cycle is already set 
      // checking >4 because 3 unique states and 1 more state to complete full cycle
      if (traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].size() > 4)
      {
        continue;
      }

      // currently expecting 1 lane, so 0th index is used only when setting states
      if (current_movement_state.movement_event_list.empty())
      {
        ROS_INFO_STREAM("Movement_event_list is empty . intersection_id: " << (int)curr_intersection.id.id << 
                           ", and signal_group_id: " << (int)current_movement_state.signal_group);
        continue;
      }

      // raw min_end_time in seconds measured from the most recent full hour
      ros::Time min_end_time(current_movement_state.movement_event_list[0].timing.min_end_time);
      
      if (curr_intersection.moy_exists) //account for minute of the year
      {
        auto inception_boost(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")); // inception of epoch
        auto duration_since_inception(boost::posix_time::seconds(ros::Time::now().toSec()));
        auto curr_time_boost = inception_boost + duration_since_inception;
        ROS_DEBUG_STREAM("Calculated current time: " << boost::posix_time::to_simple_string(curr_time_boost));
        
        int curr_year = curr_time_boost.date().year();
        auto curr_year_start_boost(boost::posix_time::time_from_string(std::to_string(curr_year)+ "-01-01 00:00:00.000"));

        ROS_DEBUG_STREAM("MOY extracted: " << (int)curr_intersection.moy);
        auto curr_minute_stamp_boost = curr_year_start_boost + boost::posix_time::minutes((int)curr_intersection.moy);
        
        int hours_of_day = curr_minute_stamp_boost.time_of_day().hours();
        int curr_month = curr_minute_stamp_boost.date().month(); 
        int curr_day = curr_minute_stamp_boost.date().day(); 

        auto curr_day_boost(boost::posix_time::time_from_string(std::to_string(curr_year) + "/" + std::to_string(curr_month) + "/" + std::to_string(curr_day) +" 00:00:00.000")); // GMT is the standard
        auto curr_hour_boost = curr_day_boost + boost::posix_time::hours(hours_of_day);

        auto curr_hour_stamp = ros::Time::fromBoost(curr_hour_boost);

        min_end_time += ros::Duration(curr_hour_stamp.toSec());
        ROS_DEBUG_STREAM("New min_end_time: " << std::to_string(min_end_time.toSec()));
      }

      //if same data as last time:
      //where state is same and timestamp is equal or less, skip
      if (traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].size() > 0 && traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().second ==
          static_cast<lanelet::CarmaTrafficLightState>(current_movement_state.movement_event_list[0].event_state.movement_phase_state) &&
          traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().first >= min_end_time) 
      {
        continue;
      }

      // if received same state as last time, but with updated time, just update the time of last state
      // skip setting state until received a new state that is different from last recorded one
      if (traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].size() > 0 && traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().second ==
          static_cast<lanelet::CarmaTrafficLightState>(current_movement_state.movement_event_list[0].event_state.movement_phase_state) &&
          traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().first < min_end_time)
      {
        ROS_DEBUG_STREAM("Updated time for state: " << static_cast<lanelet::CarmaTrafficLightState>(current_movement_state.movement_event_list[0].event_state.movement_phase_state)  << ", with time: " 
                          << std::to_string(min_end_time.toSec()));
        traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().first = min_end_time;
        continue;
      }

      // detected that new state received; therefore, set the last recorded state (not new one received)
      ROS_DEBUG_STREAM("Received new state for light: " << curr_light_id << ", with state: " << static_cast<lanelet::CarmaTrafficLightState>(current_movement_state.movement_event_list[0].event_state.movement_phase_state) <<
                        ", time: " << ros::Time(min_end_time));


      if (traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].size() >= 2 
          && traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].front().second == 
             traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().second)
        {
          ROS_DEBUG_STREAM("Setting last recorded state for light: " << curr_light_id << ", with state: " << traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().second <<
                        ", time: " << traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().first);
          curr_light->setStates(traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group], curr_intersection.revision);
          ROS_DEBUG_STREAM("Set new cycle of total seconds: " << curr_light->fixed_cycle_duration.toSec());                  
        }
      else if (curr_light->recorded_time_stamps.empty()) // if it was never initialized, do its best to plan with the current state until the future state is also received.
      {
        std::vector<std::pair<ros::Time, lanelet::CarmaTrafficLightState>> default_state;
        // green 20sec, yellow 3sec, red 20sec, back to green 20sec etc...
        default_state.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(ros::Time(0), lanelet::CarmaTrafficLightState::PROTECTED_MOVEMENT_ALLOWED));
        default_state.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(default_state.back().first + ros::Duration(YELLOW_LIGHT_DURATION), lanelet::CarmaTrafficLightState::PROTECTED_CLEARANCE));
        default_state.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(default_state.back().first + ros::Duration(RED_LIGHT_DURATION), lanelet::CarmaTrafficLightState::STOP_AND_REMAIN));
        default_state.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(default_state.back().first + ros::Duration(GREEN_LIGHT_DURATION), lanelet::CarmaTrafficLightState::PROTECTED_MOVEMENT_ALLOWED));
        curr_light->setStates(default_state, curr_intersection.revision);
        ROS_DEBUG_STREAM("Set default cycle of total seconds: " << curr_light->fixed_cycle_duration.toSec());                  

      }
      else if (traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].size() >= 2 )
      {
        ros::Duration green_light_duration = ros::Duration(GREEN_LIGHT_DURATION);
        ros::Duration yellow_light_duration = ros::Duration(YELLOW_LIGHT_DURATION);
        ros::Duration red_light_duration = ros::Duration(RED_LIGHT_DURATION);

        std::vector<std::pair<ros::Time, lanelet::CarmaTrafficLightState>> partial_states;
        // set the partial cycle.
        ROS_DEBUG_STREAM("Setting last recorded state for light: " << curr_light_id << ", with state: " << traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().second <<
              ", time: " << traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].back().first);
        for (size_t i = 0; i < traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].size() - 1; i ++)
        {
          auto light_state = traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i + 1].second;

          if (light_state == lanelet::CarmaTrafficLightState::STOP_AND_REMAIN || light_state == lanelet::CarmaTrafficLightState::STOP_THEN_PROCEED)
            red_light_duration = traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i + 1].first - traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i].first;
          
          else if (light_state == lanelet::CarmaTrafficLightState::PERMISSIVE_MOVEMENT_ALLOWED || light_state == lanelet::CarmaTrafficLightState::PROTECTED_MOVEMENT_ALLOWED)
            green_light_duration = traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i + 1].first - traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i].first;
          
          else if (light_state == lanelet::CarmaTrafficLightState::PERMISSIVE_CLEARANCE || light_state == lanelet::CarmaTrafficLightState::PROTECTED_CLEARANCE)
            yellow_light_duration = traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i + 1].first - traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group][i].first;
        }

        partial_states.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(ros::Time(0), lanelet::CarmaTrafficLightState::PROTECTED_MOVEMENT_ALLOWED));
        partial_states.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(partial_states.back().first + yellow_light_duration, lanelet::CarmaTrafficLightState::PROTECTED_CLEARANCE));
        partial_states.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(partial_states.back().first + red_light_duration, lanelet::CarmaTrafficLightState::STOP_AND_REMAIN));
        partial_states.push_back(std::make_pair<ros::Time, lanelet::CarmaTrafficLightState>(partial_states.back().first + green_light_duration, lanelet::CarmaTrafficLightState::PROTECTED_MOVEMENT_ALLOWED));
        
        curr_light->setStates(partial_states, curr_intersection.revision);
        ROS_DEBUG_STREAM("Set new partial cycle of total seconds: " << curr_light->fixed_cycle_duration.toSec());                  

      }

      // record the new state received
      traffic_light_states_[curr_intersection.id.id][current_movement_state.signal_group].push_back(std::make_pair(min_end_time, 
                              static_cast<lanelet::CarmaTrafficLightState>(current_movement_state.movement_event_list[0].event_state.movement_phase_state)));
    }
  }
}

}  // namespace carma_wm