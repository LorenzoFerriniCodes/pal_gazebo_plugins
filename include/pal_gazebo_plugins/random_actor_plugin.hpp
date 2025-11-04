// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//	 http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PAL_GAZEBO_PLUGINS__GAZEBO_UNDERACTUATED_FINGER_HPP_
#define PAL_GAZEBO_PLUGINS__GAZEBO_UNDERACTUATED_FINGER_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>

#include <cmath>
#include <random>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <hri_msgs/msg/ids_list.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class CentripetalRandomWalkActorPlugin : public ModelPlugin
{
private:
	physics::ModelPtr model;
	physics::ActorPtr actor;
	event::ConnectionPtr updateConnection;

	// Motion parameters
	double speed = 0.3;					 // Constant forward speed (m/s)
	double centripetalAccel = 0.05;		 // Centripetal acceleration (m/s^2)
	double angleSign = 1.0;				 // +1 = left, -1 = right
	double reSampleTime = 10.0;			 // Seconds between random resamples
	double timeSinceLastSample = 0.0;

	ignition::math::Vector3d direction;	 // Current movement direction
	common::Time lastUpdate;			 // Time of last update

	// Random number generation
	std::random_device rd;
	std::mt19937 gen;
	std::normal_distribution<double> accelDist;

	// Fixed square bounds
	double minX = -5.0;
	double maxX =  5.0;
	double minY = -5.0;
	double maxY =  5.0;

	rclcpp::Node::SharedPtr rosNode = nullptr;
	rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr idsPub = nullptr;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster = nullptr;

	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
	std::thread spinThread;

	bool hriSimulation = false;

public:
	CentripetalRandomWalkActorPlugin();

	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
	void OnUpdate(const common::UpdateInfo &_info);
};
}  // namespace gazebo

#endif  // PAL_GAZEBO_PLUGINS__GAZEBO_UNDERACTUATED_FINGER_HPP_
