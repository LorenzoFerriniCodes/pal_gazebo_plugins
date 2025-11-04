// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ignition/math/Pose3.hh>

#include "pal_gazebo_plugins/random_actor_plugin.hpp"

namespace gazebo
{
CentripetalRandomWalkActorPlugin::CentripetalRandomWalkActorPlugin() :
  gen(rd()), accelDist(0.05, 0.02) {}

void CentripetalRandomWalkActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->model = _model;
    this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

    if (!this->actor)
    {
        gzerr << "CentripetalRandomWalkActorPlugin requires an Actor.\n";
        return;
    }

    bool hasWorldLimits = _sdf->HasElement("world_limits");

    if (hasWorldLimits && _sdf->GetElement("world_limits")->HasElement("x_max"))
    {
        maxX = _sdf->GetElement("world_limits")->GetElement("x_max")->Get<double>();
        gzmsg << "x_max set to " << maxX << "\n";
    } else {
        gzmsg << "Using default x_max " << maxX << "\n";
    }
    if (hasWorldLimits && _sdf->GetElement("world_limits")->HasElement("x_min"))
    {
        minX = _sdf->GetElement("world_limits")->GetElement("x_min")->Get<double>();
        gzmsg << "x_min set to " << minX << "\n";
    } else {
        gzmsg << "Using default x_min " << minX << "\n";
    }
    if (hasWorldLimits && _sdf->GetElement("world_limits")->HasElement("y_max"))
    {
        maxY = _sdf->GetElement("world_limits")->GetElement("y_max")->Get<double>();
        gzmsg << "y_max set to " << maxY << "\n";
    } else {
        gzmsg << "Using default y_max " << maxY << "\n";
    }
    if (hasWorldLimits && _sdf->GetElement("world_limits")->HasElement("y_min"))
    {
        minY = _sdf->GetElement("world_limits")->GetElement("y_min")->Get<double>();
        gzmsg << "y_min set to " << minY << "\n";
    } else {
        gzmsg << "Using default y_min " << minY << "\n";
    }

    if (maxX <= minX || maxY <= minY)
    {
        gzerr << "Invalid world limits specified. Check that x_max > x_min and y_max > y_min.\n";
        return;
    }

    hriSimulation = _sdf->HasElement("hri_simulation") && _sdf->Get<bool>("hri_simulation");
    gzmsg << "ROS4HRI Simulation mode: " << (hriSimulation ? "ON" : "OFF") << "\n";

    if (hriSimulation) {
        if (!rclcpp::ok()) {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }


        rclcpp::NodeOptions nodeOptions;
        nodeOptions.arguments({"--ros-args", "-p", "use_sim_time:=true"});

        rosNode = rclcpp::Node::make_shared("centripetal_random_walk_actor_plugin_node", nodeOptions);
        idsPub = rosNode->create_publisher<hri_msgs::msg::IdsList>("/humans/bodies/tracked", 10);
        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(rosNode);

        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(rosNode);
        spinThread = std::thread([this]() {
            executor->spin();
        });
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CentripetalRandomWalkActorPlugin::OnUpdate, this, std::placeholders::_1));

    // Random initial direction
    double angle = ignition::math::Rand::DblUniform(-M_PI, M_PI);
    this->direction = ignition::math::Vector3d(std::cos(angle), std::sin(angle), 0);

    // Set walking animation if available
    auto skelAnims = this->actor->SkeletonAnimations();
    if (skelAnims.find("walking") != skelAnims.end())
    {
        physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
        trajectoryInfo->type = "walking";
        trajectoryInfo->duration = 1.0;
        this->actor->SetCustomTrajectory(trajectoryInfo);
    }

    // Start position slightly above the ground
    physics::LinkPtr canonicalLink = this->actor->GetLink();
    if (canonicalLink)
    {
        this->model->SetLinkWorldPose(
            ignition::math::Pose3d(
                ignition::math::Vector3d(0.0, 0.0, 1.0),
                ignition::math::Quaternion(1.57, 0.0, 0.0)),
            canonicalLink->GetScopedName());
    }

    this->lastUpdate = common::Time::Zero;
    gzmsg << "Centripetal random walk actor plugin (bounded) loaded.\n";
}

void CentripetalRandomWalkActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    if (this->lastUpdate == common::Time::Zero)
    {
        this->lastUpdate = _info.simTime;
        return;
    }

    double dt = (_info.simTime - this->lastUpdate).Double();
    this->lastUpdate = _info.simTime;
    this->timeSinceLastSample += dt;

    if (dt <= 0.0)
    {
        gzerr << "Non-positive dt detected in CentripetalRandomWalkActorPlugin.\n";
        return;
    }

    if (this->timeSinceLastSample > this->reSampleTime)
    {
        this->centripetalAccel = std::max(0.002, this->accelDist(this->gen));
        this->angleSign = (ignition::math::Rand::DblUniform(0, 1) > 0.5) ? 1.0 : -1.0;
        this->timeSinceLastSample = 0.0;
    }

    double angularVel = (this->centripetalAccel / this->speed) * this->angleSign;

    double deltaYaw = angularVel * dt;
    ignition::math::Quaterniond rot(0, 0, deltaYaw);
    this->direction = rot.RotateVector(this->direction);
    this->direction.Normalize();

    ignition::math::Pose3d pose = this->actor->WorldPose();
    ignition::math::Vector3d displacement = this->direction * this->speed * dt;
    ignition::math::Vector3d newPos = pose.Pos() + displacement;
    newPos.Z(1.0);

    bool bounced = false;

    if (newPos.X() < minX)
    {
        newPos.X(minX);
        this->direction.X(fabs(this->direction.X()));
        bounced = true;
    }
    else if (newPos.X() > maxX)
    {
        newPos.X(maxX);
        this->direction.X(-fabs(this->direction.X()));
        bounced = true;
    }

    if (newPos.Y() < minY)
    {
        newPos.Y(minY);
        this->direction.Y(fabs(this->direction.Y()));
        bounced = true;
    }
    else if (newPos.Y() > maxY)
    {
        newPos.Y(maxY);
        this->direction.Y(-fabs(this->direction.Y()));
        bounced = true;
    }

    if (bounced)
    {
        double randomAngle = ignition::math::Rand::DblUniform(-M_PI / 6, M_PI / 6);
        ignition::math::Quaterniond rot(0, 0, randomAngle);
        this->direction = rot.RotateVector(this->direction);
    }

    double yaw = atan2(this->direction.Y(), this->direction.X());
    yaw += M_PI_2; // offset for actor model orientation
    ignition::math::Quaterniond newRot(1.57, 0.0, yaw);
    ignition::math::Quaterniond rotForTransform(0.0, 0.0, yaw - M_PI_2);

    ignition::math::Pose3d newPose(newPos, newRot);

    physics::LinkPtr canonicalLink = this->actor->GetLink();
    if (canonicalLink)
    {
        this->model->SetLinkWorldPose(newPose, canonicalLink->GetScopedName());
    }

    this->actor->SetScriptTime(this->actor->ScriptTime() + dt);
    this->actor->Update();

    if (hriSimulation) {
        hri_msgs::msg::IdsList idsMsg;
        idsMsg.ids.push_back("gazeb");
        idsMsg.header.stamp = rosNode->now() - rclcpp::Duration::from_seconds(0.5);
        idsPub->publish(idsMsg);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = rosNode->now();
        t.header.frame_id = "map";
        t.child_frame_id = "body_gazeb";
        t.transform.translation.x = newPos.X();
        t.transform.translation.y = newPos.Y();
        t.transform.translation.z = newPos.Z();
        t.transform.rotation.x = rotForTransform.X();
        t.transform.rotation.y = rotForTransform.Y();
        t.transform.rotation.z = rotForTransform.Z();
        t.transform.rotation.w = rotForTransform.W();
        tfBroadcaster->sendTransform(t);
    }
}

GZ_REGISTER_MODEL_PLUGIN(CentripetalRandomWalkActorPlugin)
}  // namespace gazebo
