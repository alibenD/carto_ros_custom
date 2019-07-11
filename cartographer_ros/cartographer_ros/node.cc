/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>
#include <fstream>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace {

cartographer_ros_msgs::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  topics.nav_sat_fix_topic = kNavSatFixTopic;
  topics.landmark_topic = kLandmarkTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
  InitVisualization();

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);


  landmarks_point_cloud_publisher_ =
      node_handle_.advertise<visualization_msgs::Marker>(
          "Landmarks", 1);

  landmarks_point_cloud_history_publisher_ =
      node_handle_.advertise<visualization_msgs::Marker>(
          "LandmarksHistory", 1);

  landmarks_point_cloud_global_history_publisher_ =
      node_handle_.advertise<visualization_msgs::Marker>(
          "LandmarksHistoryGlobal", 1);

//          landmark_local_.reserve(10000);
          landmark_local_with_time_.reserve(10000);

  point_cloud_compare_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("Compara_Scan", 1);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
}

Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return true;
}

bool Node::InitVisualization() {
  current_landmark_array_.ns = "current_frame_landmarks";
  current_landmark_array_.header.frame_id = "map";
  current_landmark_array_.action = visualization_msgs::Marker::ADD;
  current_landmark_array_.type = visualization_msgs::Marker::LINE_LIST;
  current_landmark_array_.id = 1;
  current_landmark_array_.color.r = 0.6;
  current_landmark_array_.color.g = 0.3;
  current_landmark_array_.color.b = 0.0;
  current_landmark_array_.color.a = 1.0;
  current_landmark_array_.scale.x = 0.01;

  history_landmark_array_.ns = "history_frame_landmarks";
  history_landmark_array_.header.frame_id = "map";
  history_landmark_array_.action = visualization_msgs::Marker::ADD;
  history_landmark_array_.type = visualization_msgs::Marker::LINE_LIST;
  history_landmark_array_.id = 2;
  history_landmark_array_.color.r = 0.9;
  history_landmark_array_.color.g = 0.6;
  history_landmark_array_.color.b = 0.3;
  history_landmark_array_.color.a = 1.0;
  history_landmark_array_.scale.x = 0.01;

  history_global_landmark_array_.ns = "history_frame_landmarks";
  history_global_landmark_array_.header.frame_id = "map";
  history_global_landmark_array_.action = visualization_msgs::Marker::ADD;
  history_global_landmark_array_.type = visualization_msgs::Marker::LINE_LIST;
  history_global_landmark_array_.id = 3;
  history_global_landmark_array_.color.r = 0.2;
  history_global_landmark_array_.color.g = 0.3;
  history_global_landmark_array_.color.b = 0.9;
  history_global_landmark_array_.color.a = 1.0;
  history_global_landmark_array_.scale.x = 0.01;
  return true;
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  const auto&& ts = map_builder_bridge_.GetTrajectoryStates();
  for (const auto& entry : ts) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0 || node_options_.generate_landmark == true) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_state.local_slam_data
                                ->range_data_in_local.returns.size());
        static bool flag_comp = false;
        for (const Eigen::Vector3f point :
             trajectory_state.local_slam_data->range_data_in_local.returns) {
          Eigen::Vector4f point_time;
          point_time << point, 0.f;
          point_cloud.push_back(point_time);
        }
        auto result = [&trajectory_state, this, &ts]() -> bool {
            if(trajectory_state.local_slam_data == nullptr)
            {
              ROS_ERROR_STREAM("local_slam_data is nullptr");
              return false;
            }
            auto timestamp_laser = ToRos(std::get<1>(latest_pl_));
            auto timestamp_pl_matched = ToRos(trajectory_state.local_slam_data->time);
            if(std::abs((timestamp_laser - timestamp_pl_matched).toSec()) <= 0.010)
            {
//              ROS_WARN_STREAM("Sync with local");
              auto& pointcloud = std::get<0>(latest_pl_);
              size_t point_size = pointcloud.points.size();

              current_landmark_array_.header.stamp = timestamp_laser;
              history_landmark_array_.header.stamp = timestamp_laser;
              history_global_landmark_array_.header.stamp = timestamp_laser;

              {
                if(ts.empty() == true)
                {
                  ROS_ERROR_STREAM("TS EMPTY.");
                  return false;
                }
                else
                {
                  flag_comp = true;
                  int trajectory_id = 3;
                  // auto const& traj_state = map_builder_bridge_.GetTrajectoryStates()[trajectory_id];
                  if(trajectory_state.local_slam_data == nullptr)
                  {
                    ROS_ERROR_STREAM("local_slam_data is nullptr");
                    return false;
                  }
                  auto& local_pose = trajectory_state.local_slam_data->local_pose;
                  auto& local_to_map = trajectory_state.local_to_map;

                  // // ROS_INFO_STREAM(sensor_to_tracking->cast<float>().DebugString());
                  // auto& trajectories = this->map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodes().trajectories();
                  // const auto it = trajectory.data_.find(GetIndex(id));

                  // const auto& trajectory_nodes = this->map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodes();
                  // const auto& trajectories = trajectory_nodes.trajectories()[0].data_;
                  if(map_builder_bridge_.map_builder_ == nullptr
                     || map_builder_bridge_.map_builder_->pose_graph() == nullptr)
                  {
                    ROS_ERROR_STREAM("Builder nullptr");
                    return false;
                  }

//                  auto traj_lists = map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodes().trajectories_;
//                  if(traj_lists.empty() == true) return false;
//                  auto& traj = traj_lists.at(0);
//
//                  auto num_traj_nodes = traj.data_.size();
//                  auto last_traj_node = traj.data_[num_traj_nodes-1];
//                  if(last_traj_node.constant_data == nullptr) return false;
//                  auto time_last_node = ToRos(last_traj_node.constant_data->time);
                  // ROS_INFO_STREAM(local_pose.cast<float>().DebugString());
                  // ROS_WARN_STREAM("LocalPose(Latest): " << num_traj_nodes);
                  // ROS_WARN_STREAM("Latest Pose: " << last_traj_node.second.const);
                  // ROS_WARN_STREAM("Handler------PoseGraph: " << node.map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodes().size());

                  // auto global_pose = last_traj_node.global_pose;
                  //local_pose.DebugString();


                  CHECK_GE(trajectory_id, 0);
//                  auto trajectory = map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodes().trajectories_[trajectory_id];
                    auto trajectory = map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodePoses().trajectories_[trajectory_id];
//                    ROS_ERROR_STREAM("Traj Pose: " << traj_poses.size());
//                  if(trajectory.can_append_ == false) return false;
                  const int index = trajectory.data_.empty() ? -1 : trajectory.data_.rbegin()->first;
//                  ROS_WARN_STREAM("TrajNode idx: " << index);
//                  if(trajectory.data_.find(index) == trajectory.data_.end())

                  if(index < 0)
                  {
                    ROS_WARN_STREAM("No traj node found: " << index);
                    return false;
                  }
                  if(trajectory.data_[index].constant_pose_data.has_value() == false){ return false;}
//                  auto time_last_node = ToRos(trajectory.data_[index].constant_pose_data.value().time);

                  auto const& sensor_bridge = map_builder_bridge_.sensor_bridge(trajectory_id);
                  auto const& tf_buf = sensor_bridge->tf_bridge();
                  std::string laser_frame_id = "laser";
                  auto sensor_to_tracking = tf_buf.LookupToTracking(std::get<1>(latest_pl_), [](auto& frame_id){
                      if(frame_id.size() > 0){
                        CHECK_NE(frame_id[0], '/') << "Should not start with a /.";
                      }
                      return frame_id;
                  }(laser_frame_id));

                  // auto global_result = pointcloud;

                  auto transformed_pointcloud = [&](decltype(pointcloud)& pl_data, decltype(sensor_to_tracking)& stt){
//                      auto result = pl_data;
                      cartographer::sensor::PointCloudWithIntensities result;
                      result.intensities = pl_data.intensities;

                      result.points.clear();
                      result.points.reserve(pl_data.points.size());
                      // global_result.points.clear();
                      // global_result.points.reserve(pl_data.points.size());

                      for(const Eigen::Vector4f& point : pl_data.points)
                      {
                        Eigen::Vector4f result_point;
                        // Eigen::Vector4f result_global_point;
                        //auto tf_to_map = local_pose * (*stt);
                        //result_point.head<3>() = stt->cast<float>() * point.head<3>();
                        result_point.head<3>() = (local_to_map * local_pose * (*stt)).cast<float>() * point.head<3>();
                        // result_point.head<3>() = (local_pose * (*stt)).cast<float>() * point.head<3>();
                        result_point[3] = point[3];
                        result.points.emplace_back(result_point);

                        // result_global_point.head<3>() = (global_pose* local_to_map * (*stt)).cast<float>() * point.head<3>();
                        // result_global_point[3] = point[3];
                        // global_result.points.emplace_back(result_global_point);
                      }
                      result.intensities = pl_data.intensities;
                      // global_result.intensities = pl_data.intensities;

                      return result;
                  }(pointcloud, sensor_to_tracking);

                  static std::vector<Eigen::Vector3f> current_landmarks;
                  current_landmarks.clear();
                  current_landmark_array_.points.clear();
                  for(size_t idx = 0; idx < point_size; ++idx)
                  {
                    auto point_stamp = pointcloud.points[idx];
                    auto point_map_stamp = transformed_pointcloud.points[idx];

                    // auto point_global_stamp = global_result.points[idx];

                    Eigen::Vector3f point(point_stamp[0], point_stamp[1], point_stamp[2]);
                    Eigen::Vector3f point_map(point_map_stamp[0], point_map_stamp[1], point_map_stamp[2]);
                    // Eigen::Vector3f point_global(point_global_stamp[0], point_global_stamp[1], point_global_stamp[2]);
                    if(point.norm() > 0.5 && point.norm() < 12 && std::abs(point[2]) <=0.2)
                    {
                      //ROS_ERROR_STREAM("In range.");
                      if(pointcloud.intensities[idx] > 1400)
                      {
                        current_landmarks.emplace_back(point);
                        tf::Vector3 landmark_point(point_map[0],point_map[1], point_map[2]);
                        geometry_msgs::PointStamped p_map;
                        p_map.header.stamp = current_landmark_array_.header.stamp;
                        p_map.header.frame_id = current_landmark_array_.header.frame_id;
                        p_map.point.x = landmark_point[0];
                        p_map.point.y = landmark_point[1];
                        p_map.point.z = landmark_point[2];
                        current_landmark_array_.points.emplace_back(p_map.point);
                        history_landmark_array_.points.emplace_back(p_map.point);
                        p_map.point.z = 2.0;
                        current_landmark_array_.points.emplace_back(p_map.point);
                        history_landmark_array_.points.emplace_back(p_map.point);
                        //ROS_WARN_STREAM("Detected.");
                        // if(std::abs((timestamp_laser-time_last_node).toSec()) < 0.012)
                        // {
                        //   tf::Vector3 landmark_point_global(point_global[0], point_global[1], point_global[2]);
                        //   geometry_msgs::PointStamped p_global;
                        //   p_global.header.stamp = current_landmark_array_.header.stamp;
                        //   p_global.header.frame_id = current_landmark_array_.header.frame_id;
                        //   p_global.point.x = landmark_point_global[0];
                        //   p_global.point.y = landmark_point_global[1];
                        //   p_global.point.z = landmark_point_global[2];
                        //   history_global_landmark_array_.points.emplace_back(p_global.point);
                        //   p_global.point.z = 2.0;
                        //   history_global_landmark_array_.points.emplace_back(p_global.point);
                        // }
                        // else{
                        //   ROS_ERROR_STREAM("Global Timeout");
                        // }
                        //localization_3.bag.fixed_odom.bag
                      }
                    }
                  }

                  landmarks_point_cloud_publisher_.publish(current_landmark_array_);
                  landmarks_point_cloud_history_publisher_.publish(history_landmark_array_);

//                  landmark_local_.emplace_back(std::make_pair(num_traj_nodes-1, current_landmarks));
                  landmark_local_with_time_.emplace_back(std::make_tuple(index, timestamp_pl_matched,current_landmarks));

//                  ROS_WARN_STREAM("Publish");


                  point_cloud_compare_publisher_.publish(ToPointCloud2Message(
                          cartographer::common::ToUniversal(trajectory_state.local_slam_data->time),
                          node_options_.map_frame,
                          cartographer::sensor::TransformTimedPointCloud(
                                  pointcloud.points, (trajectory_state.local_to_map * local_pose * (*sensor_to_tracking)).cast<float>())));
                  // ROS_WARN_STREAM("Sync_Get Laser Timestamp: " << timestamp_laser);
                  // ROS_ERROR_STREAM("Sync_Pub PCLTime: " << ToRos(trajectory_state.local_slam_data->time));
                  // static size_t count = 0;
                  // if(count<10)
                  // {
                  //   ++count;
                  //   ROS_WARN_STREAM("Count: " << count);
                  // }
                  // else
                  // {
                  //   count = 0;
                  if(flag_comp == true)
                  {
                    history_global_landmark_array_.points.clear();
//                    ROS_INFO_STREAM("Global Size: " << landmark_local_with_time_.size());
                    for(auto& landmark_pair:landmark_local_with_time_)
                    {
                      auto& node_idx = std::get<0>(landmark_pair);
                      auto& time_stamp = std::get<1>(landmark_pair);
                      auto& landmarks = std::get<2>(landmark_pair);

                      if(node_idx > index) continue;
//                      if(trajectory.can_append_ == false) continue;
//                      if(trajectory.data_.find(node_idx) == trajectory.data_.end()) continue;
//                      if(trajectory.data_[node_idx].constant_pose_data == nullptr) continue;
                        if(trajectory.data_[node_idx].constant_pose_data.has_value() == false){ ROS_ERROR_STREAM("NO VALUE!!!");continue;}
                        auto traj_time = ToRos(trajectory.data_[node_idx].constant_pose_data.value().time);
//                        ROS_ERROR_STREAM("tmp_timestamp: " << (time_stamp).toSec());
//                        ROS_ERROR_STREAM("traj_timestamp: " << (traj_time).toSec());
                      if(std::abs((time_stamp-traj_time).toSec()) >= 0.010) { /*ROS_ERROR_STREAM("Timeout: " << (time_stamp-traj_time).toSec());*/ continue;}
                      // auto node_global_pose = trajectories[node_idx].constant_data->local_pose;
                      auto& node_global_pose = trajectory.data_[node_idx].global_pose;
//                       ROS_WARN_STREAM("Update Global");
                      for(const Eigen::Vector3f& landmark_laser:landmarks)
                      {
                        Eigen::Vector3f global_lm = (node_global_pose * (*sensor_to_tracking)).cast<float>() * landmark_laser;

                        tf::Vector3 landmark_point_global(global_lm[0], global_lm[1], global_lm[2]);
                        geometry_msgs::PointStamped p_global;
                        p_global.header.stamp = current_landmark_array_.header.stamp;
                        p_global.header.frame_id = current_landmark_array_.header.frame_id;
                        p_global.point.x = landmark_point_global[0];
                        p_global.point.y = landmark_point_global[1];
                        p_global.point.z = landmark_point_global[2];
                        history_global_landmark_array_.points.emplace_back(p_global.point);
                        p_global.point.z = 2.0;
                        history_global_landmark_array_.points.emplace_back(p_global.point);
                      }
                      flag_comp = false;
                    }
                    landmarks_point_cloud_global_history_publisher_.publish(history_global_landmark_array_);

                  }
                }
              }
            }//END
            return true;
        }();
        // else
        // {
        //    cartographer::common::MutexLocker lock(&mutex_pl_);
        //    ROS_ERROR_STREAM("DROP, TIMEOUT; Diff: " << (timestamp_laser-timestamp_pl_matched).toSec());
        //    ROS_ERROR_STREAM("Size: " << buff_pl_.size());
        //    for(auto& pl: buff_pl_)
        //    {
        //      auto buff_timestamp = ToRos(std::get<1>(pl));
        //      if(buff_timestamp == timestamp_pl_matched)
        //      {
        //        ROS_WARN_STREAM("Found!!!!!!!!!!!!!!!!!!!");
        //      }
        //      ROS_WARN_STREAM("DROP, TIMEOUT; Diff: " << (buff_timestamp-timestamp_pl_matched).toSec());
        //    }
        // }


        scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_state.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_state.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_state.local_slam_data->time,
                           trajectory_state.local_slam_data->local_pose);
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp = ToRos(now);

    const Rigid3d tracking_to_local = [&] {
      if (trajectory_state.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(extrapolator.ExtrapolatePose(now)));
      }
      return extrapolator.ExtrapolatePose(now);
    }();

    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));
        tf_broadcaster_.sendTransform(stamped_transform);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}

void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::SensorTopics& topics) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(
        SensorId{SensorType::ODOMETRY, topics.odometry_topic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    std::string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, topic,
                                                &node_handle_, this),
         topic});
  }

  if (options.use_odometry) {
    std::string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, topic,
                                                  &node_handle_, this),
         topic});
  }
  if (options.use_nav_sat) {
    std::string topic = topics.nav_sat_fix_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  if (options.use_landmarks) {
    std::string topic = topics.landmark_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options, topics)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

void Node::saveLandmarks(const std::string& save_path)
{
    std::ofstream landmark_hd;
    landmark_hd.open(save_path);
      auto trajectory = map_builder_bridge_.map_builder_->pose_graph()->GetTrajectoryNodePoses().trajectories_[0];
    const int index = trajectory.data_.empty() ? -1 : trajectory.data_.rbegin()->first;

    if(index < 0)
    {
        ROS_WARN_STREAM("No traj node found: " << index);
        exit(1);
    }
    auto const& sensor_bridge = map_builder_bridge_.sensor_bridge(3);
    auto const& tf_buf = sensor_bridge->tf_bridge();
    std::string laser_frame_id = "laser";
    auto sensor_to_tracking = tf_buf.LookupToTracking(std::get<1>(latest_pl_), [](auto& frame_id){
        if(frame_id.size() > 0){
            CHECK_NE(frame_id[0], '/') << "Should not start with a /.";
        }
        return frame_id;
    }(laser_frame_id));
    size_t size_landmark = 0;
    for(auto& landmark_pair:landmark_local_with_time_)
    {
        auto& node_idx = std::get<0>(landmark_pair);
        auto& time_stamp = std::get<1>(landmark_pair);
        auto& landmarks = std::get<2>(landmark_pair);

        if(node_idx > index) continue;
        if(trajectory.data_[node_idx].constant_pose_data.has_value() == false){ ROS_ERROR_STREAM("NO VALUE!!!");continue;}
        auto traj_time = ToRos(trajectory.data_[node_idx].constant_pose_data.value().time);
        if(std::abs((time_stamp-traj_time).toSec()) >= 0.010) { /*ROS_ERROR_STREAM("Timeout: " << (time_stamp-traj_time).toSec());*/ continue;}
        auto& node_global_pose = trajectory.data_[node_idx].global_pose;
        size_landmark += landmarks.size();
        for(const Eigen::Vector3f& landmark_laser:landmarks)
        {
            Eigen::Vector3f global_lm = (node_global_pose * (*sensor_to_tracking)).cast<float>() * landmark_laser;

//            tf::Vector3 landmark_point_global(global_lm[0], global_lm[1], global_lm[2]);
            landmark_hd << global_lm[0] << " " << global_lm[1] << " " << global_lm[2] << std::endl;
        }
    }
    landmark_hd.close();
    ROS_WARN_STREAM("Landmarks saved, in total " << size_landmark << ". Saved at " << save_path);
}

cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;

  // First, check if we can actually finish the trajectory.
  if (map_builder_bridge_.GetFrozenTrajectoryIds().count(trajectory_id)) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    status_response.message = error;
    return status_response;
  }
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    status_response.message = error;
    return status_response;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                              " has already been finished.";
    LOG(ERROR) << error;
    status_response.code =
        cartographer_ros_msgs::StatusCode::RESOURCE_EXHAUSTED;
    status_response.message = error;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  for (auto& entry : subscribers_[trajectory_id]) {
    entry.subscriber.shutdown();
    subscribed_topics_.erase(entry.topic);
    LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  CHECK(is_active_trajectory_.at(trajectory_id));
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  const std::string message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  status_response.message = message;
  return status_response;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request.options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    const std::string error = "Invalid trajectory options.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else if (!ValidateTopicNames(request.topics, options)) {
    const std::string error = "Invalid topics.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else {
    response.trajectory_id = AddTrajectory(options, request.topics);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "Success.";
  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id :
         ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics())) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  if (map_builder_bridge_.SerializeState(request.filename)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "State written to '" + request.filename + "'.";
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = "Failed to write '" + request.filename + "'.";
  }
  return true;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : is_active_trajectory_) {
      CHECK(!entry.second);
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(map_builder_bridge_.SerializeState(filename))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}

}  // namespace cartographer_ros
