// === Minimal, fuel-tools–free includes ===
#include <gazebo/common/Events.hh> // event::Events::ConnectWorldUpdateBegin
#include <gazebo/common/Plugin.hh> // ModelPlugin
#include <gazebo/common/Time.hh>   // common::Time
#include <gazebo/physics/Joint.hh> // physics::Joint
#include <gazebo/physics/Model.hh> // physics::Model
#include <gazebo/physics/World.hh> // physics::World
#include <ignition/math/Vector3.hh>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

inline double planarDist(const ignition::math::Vector3d &a,
                         const ignition::math::Vector3d &b) {
  const double dx = a.X() - b.X();
  const double dy = a.Y() - b.Y();
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace

namespace gazebo {

class AvoiderPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model_->GetWorld();

    // Read SDF params if provided
    if (sdf) {
      if (sdf->HasElement("safe_distance"))
        safeDist_ = sdf->Get<double>("safe_distance");
      if (sdf->HasElement("turn_time_ms"))
        turnMs_ = sdf->Get<int>("turn_time_ms");
      if (sdf->HasElement("speed"))
        driveSpeed_ = sdf->Get<double>("speed");
      if (sdf->HasElement("left_joint"))
        leftJointName_ = sdf->Get<std::string>("left_joint");
      if (sdf->HasElement("right_joint"))
        rightJointName_ = sdf->Get<std::string>("right_joint");
    }

    left_ = model_->GetJoint(leftJointName_);
    right_ = model_->GetJoint(rightJointName_);
    if (!left_ || !right_) {
      std::cerr << "[avoider] ERROR: joints not found. left=" << leftJointName_
                << " right=" << rightJointName_ << std::endl;
      return;
    }

    std::cout << "[Gazebo] AvoiderPlugin loaded\n";
    std::cout << "[avoider] Params: safeDist=" << safeDist_
              << " turnMs=" << turnMs_ << " speed(rad/s)=" << driveSpeed_
              << std::endl;

    // Obstacle models present in pro_world.world
    obstacleNames_ = {"box_red", "box_green", "low_wall"};

    state_ = STATE_FORWARD;
    lastSwitch_ = world_->SimTime();

    updateConn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AvoiderPlugin::OnUpdate, this));
  }

private:
  // استخدمت enum عادي (مش enum class) لتجنب تحذير clang “Reference to
  // enumeration …”
  enum State { STATE_FORWARD, STATE_BACKING, STATE_TURNING };

  void setWheels(double vl, double vr) {
    // index 0 هو محور الدوران الوحيد في joint
    left_->SetVelocity(0, vl);
    right_->SetVelocity(0, vr);
  }

  double minObstacleDist() {
    const auto pos = model_->WorldPose().Pos();
    double minD = 1e9;
    for (const auto &name : obstacleNames_) {
      auto m = world_->ModelByName(name);
      if (!m)
        continue;
      minD = std::min(minD, planarDist(pos, m->WorldPose().Pos()));
    }
    return minD;
  }

  void OnUpdate() {
    if (!left_ || !right_)
      return;

    const double d = minObstacleDist();
    const common::Time now = world_->SimTime();

    switch (state_) {
    case STATE_FORWARD:
      if (d < safeDist_) {
        state_ = STATE_BACKING;
        lastSwitch_ = now;
        setWheels(-0.5 * driveSpeed_, -0.5 * driveSpeed_);
      } else {
        setWheels(driveSpeed_, driveSpeed_);
      }
      break;

    case STATE_BACKING:
      if ((now - lastSwitch_).Double() * 1000.0 >= backMs_) {
        state_ = STATE_TURNING;
        lastSwitch_ = now;
        setWheels(-0.6 * driveSpeed_, +0.6 * driveSpeed_);
      }
      break;

    case STATE_TURNING:
      if ((now - lastSwitch_).Double() * 1000.0 >= turnMs_) {
        state_ = STATE_FORWARD;
        lastSwitch_ = now;
      }
      break;
    }
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr left_, right_;
  event::ConnectionPtr updateConn_;

  // tunables
  std::string leftJointName_ = "left_wheel_joint";
  std::string rightJointName_ = "right_wheel_joint";
  double safeDist_ = 0.28;  // m
  int backMs_ = 300;        // ms
  int turnMs_ = 420;        // ms
  double driveSpeed_ = 2.2; // rad/s

  std::vector<std::string> obstacleNames_;
  common::Time lastSwitch_;
  State state_;
};

// سجل البلجن كـ ModelPlugin (مش SensorPlugin)
GZ_REGISTER_MODEL_PLUGIN(AvoiderPlugin)

} // namespace gazebo
