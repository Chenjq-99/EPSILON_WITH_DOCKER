#include "phy_simulator/visualizer.h"
#include <json/json.hpp>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
namespace phy_simulator {

using Json = nlohmann::json;

Visualizer::Visualizer(ros::NodeHandle nh) : nh_(nh) {
  vehicle_set_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/vehicle_set_vis", 10);
  lane_net_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/lane_net_vis", 10);
  obstacle_set_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/obstacle_set_vis", 10);
}

void Visualizer::VisualizeData() {
  auto time_stamp = ros::Time::now();
  VisualizeDataWithStamp(time_stamp);
  // SendTfWithStamp(time_stamp);
}

void Visualizer::VisualizeDataWithStamp(const ros::Time &stamp) {
  VisualizeVehicleSet(stamp, p_phy_sim_->vehicle_set());
  VisualizeLaneNet(stamp, p_phy_sim_->lane_net());
  VisualizeObstacleSet(stamp, p_phy_sim_->obstacle_set());
}

// void Visualizer::SendTfWithStamp(const ros::Time &stamp) {
//   auto vehicle_set = p_phy_sim_->vehicle_set();
//   for (auto iter = vehicle_set.vehicles.begin();
//        iter != vehicle_set.vehicles.end(); ++iter) {
//     static tf::TransformBroadcaster tf_broadcaster;
//     Vec3f state = iter->second.Ret3DofState();
//     geometry_msgs::Pose pose;
//     common::VisualizationUtil::GetRosPoseFrom3DofState(state, &pose);

//     std::string tf_name = std::string("body_") + std::to_string(iter->first);
//     tf_broadcaster.sendTransform(tf::StampedTransform(
//         tf::Transform(
//             tf::Quaternion(pose.orientation.x, pose.orientation.y,
//                            pose.orientation.z, pose.orientation.w),
//             tf::Vector3(pose.position.x, pose.position.y, pose.position.z)),
//         stamp, "map", tf_name));
//   }
// }

void Visualizer::VisualizeVehicleSet(const ros::Time &stamp,
                                     const common::VehicleSet &vehicle_set) {
  visualization_msgs::MarkerArray vehicle_marker;
  common::ColorARGB color_obb(1.0, 0.8, 0.8, 0.8);
  common::ColorARGB color_vel_vec(1.0, 1.0, 0.0, 0.0);
  common::ColorARGB color_steer(1.0, 1.0, 1.0, 1.0);
  for (auto iter = vehicle_set.vehicles.begin();
       iter != vehicle_set.vehicles.end(); ++iter) {
    common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
        iter->second, common::ColorARGB(1, 1, 0, 0), color_vel_vec, color_steer,
        7 * iter->first, &vehicle_marker);
  }
  //   SendTfWithStamp(stamp);
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &vehicle_marker);
  vehicle_set_pub_.publish(vehicle_marker);
}

void Visualizer::VisualizeLaneNet(const ros::Time &stamp,
                                  const common::LaneNet &lane_net) {
  std::string boundary_path = "/home/chengs/work_ws/src/EPSILON/core/playgrounds/highway_v1.0/boundary.json";
  std::fstream fs(boundary_path);
  Json root;
  fs >> root;
  Json lines = root["lines"];
  visualization_msgs::MarkerArray lane_net_marker;
  auto color = common::cmap.at("white");
  for(int i = 0;i<static_cast<int>(lines.size());i++) {
    visualization_msgs::Marker lane_marker;
    Json lane_points = lines[i]["lane"];
    int num_pts = static_cast<int>(lane_points.size());
    std::string lane_type = lines[i]["type"].get<std::string>();
    lane_marker.header.stamp = stamp;
    lane_marker.header.frame_id = "map";
    lane_marker.id = i;
    //std::cout<<"lane type is : "<<lane_type<<std::endl;
    if(lane_type == "solid") 
      lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
    else
      lane_marker.type = visualization_msgs::Marker::LINE_LIST;
    lane_marker.color.a = color.a;
    lane_marker.color.r = color.r;
    lane_marker.color.g = color.g;
    lane_marker.color.b = color.b;
    lane_marker.action = visualization_msgs::Marker::MODIFY;
    for(int k = 0;k<num_pts;k++) {
      double x = lane_points[k][0].get<double>();
      double y = lane_points[k][1].get<double>();
      geometry_msgs::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0.0;
      if(lane_type == "solid") {
        lane_marker.points.push_back(pt);
      } else {
        float heading = 0;
        if(k != num_pts - 1) {
          double next_x = lane_points[k + 1][0].get<double>();
          double next_y = lane_points[k + 1][1].get<double>();
          heading = std::atan2(next_y - y,next_x - x);
          lane_marker.points.push_back(pt);
          double x_offset = 3 * std::cos(heading);
          double y_offset = 3 * std::sin(heading);
          double dist_p = (next_x - pt.x)*(next_x - pt.x) + (next_y - pt.y)*(next_y - pt.y);
          while(dist_p>9) {
           // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"distance is : %f",dist_p);
            pt.x = pt.x + x_offset*2/3;
            pt.y = pt.y + y_offset*2/3;
            pt.z = pt.z;
            lane_marker.points.push_back(pt);
            pt.x = pt.x + x_offset/3;
            pt.y = pt.y + y_offset/3;
            pt.z = pt.z;
            lane_marker.points.push_back(pt);
            dist_p = (next_x - pt.x)*(next_x - pt.x) + (next_y - pt.y)*(next_y - pt.y);;
          }
          pt.x = next_x;
          pt.y = next_y;
          lane_marker.points.push_back(pt);
        }
      }
    }
    lane_marker.scale.x = 0.1;
    lane_net_marker.markers.push_back(lane_marker);
  }
  //visualization_msgs::MarkerArray lane_net_marker;
  //int id_cnt = 0;
  //for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
  //     ++iter) {
  //  visualization_msgs::Marker lane_marker;
  //  // common::ColorARGB(1.0, 0.0, 1.0, 1.0)
  //  common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVec(
  //      iter->second.lane_points, common::cmap.at("sky blue"),
  //      Vec3f(0.1, 0.1, 0.1), iter->second.id, &lane_marker);
  //  lane_marker.header.stamp = stamp;
  //  lane_marker.header.frame_id = "map";
  //  lane_marker.id = id_cnt++;
  //  lane_net_marker.markers.push_back(lane_marker);
  //  // Visualize the start and end point
  //  visualization_msgs::Marker start_point_marker, end_point_marker,
  //      lane_id_text_marker;
  //  {
  //    start_point_marker.header.stamp = stamp;
  //    start_point_marker.header.frame_id = "map";
  //    Vec2f pt = *(iter->second.lane_points.begin());
  //    common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
  //        Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
  //        Vec3f(0.5, 0.5, 0.5), id_cnt++, &start_point_marker);
  //    lane_id_text_marker.header.stamp = stamp;
  //    lane_id_text_marker.header.frame_id = "map";
  //    common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
  //        Vec3f(pt(0), pt(1), 0.5), std::to_string(iter->second.id),
  //        common::ColorARGB(1.0, 0.0, 0.0, 1.0), Vec3f(0.6, 0.6, 0.6), id_cnt++,
  //        &lane_id_text_marker);
  //  }
  //  {
  //    end_point_marker.header.stamp = stamp;
  //    end_point_marker.header.frame_id = "map";
  //    Vec2f pt = *(iter->second.lane_points.rbegin());
  //    common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
  //        Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
  //        Vec3f(0.5, 0.5, 0.5), id_cnt++, &end_point_marker);
  //  }
//
  //  lane_net_marker.markers.push_back(start_point_marker);
  //  lane_net_marker.markers.push_back(end_point_marker);
  //  lane_net_marker.markers.push_back(lane_id_text_marker);
  //}
  lane_net_pub_.publish(lane_net_marker);
}

void Visualizer::VisualizeObstacleSet(const ros::Time &stamp,
                                      const common::ObstacleSet &Obstacle_set) {
  visualization_msgs::MarkerArray Obstacles_marker;
  common::VisualizationUtil::GetRosMarkerUsingObstacleSet(Obstacle_set,
                                                          &Obstacles_marker);
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &Obstacles_marker);
  obstacle_set_pub_.publish(Obstacles_marker);
}

}  // namespace phy_simulator
