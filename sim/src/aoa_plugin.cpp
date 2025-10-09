#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/msgs.hh> // ✅ مهم: أنواع ignition::msgs::Marker

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

using namespace gazebo;

class AOAPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    this->model = model;
    this->world = model->GetWorld();
    this->physics = world->Physics();

    // ===== Params from SDF =====
    leftJointName = getSdf<std::string>(sdf, "left_joint", "left_hinge");
    rightJointName = getSdf<std::string>(sdf, "right_joint", "right_hinge");

    wheelRadius = getSdf<double>(sdf, "wheel_radius", 0.05);
    wheelSep = getSdf<double>(sdf, "wheel_separation", 0.24);
    safeDistCm = getSdf<double>(sdf, "safe_distance_cm", 30.0);
    fwdSpeed = getSdf<double>(sdf, "forward_speed", 0.7);
    turnSpeed = getSdf<double>(sdf, "turn_speed", 1.0);

    rayLenM = getSdf<double>(sdf, "ray_length_m", 2.0);
    leftDeg = getSdf<double>(sdf, "left_ray_deg", 30.0);
    rightDeg = getSdf<double>(sdf, "right_ray_deg", -30.0);

    vizRays = getSdf<bool>(sdf, "viz_rays", true);
    logCsv = getSdf<bool>(sdf, "log_csv", true);
    logPath = getSdf<std::string>(sdf, "log_path", "logs/aoa_log.csv");

    // ===== Joints =====
    left = model->GetJoint(leftJointName);
    right = model->GetJoint(rightJointName);
    if (!left || !right) {
      gzerr << "[AOA] ❌ Wheel joints not found. Check joint names.\n";
      return;
    }

    // ===== Transport for Marker (IGNITION msgs) =====
    if (vizRays) {
      node = transport::NodePtr(new transport::Node());
      node->Init(world->Name());
      // ✅ استخدم ignition::msgs::Marker
      markerPub = node->Advertise<ignition::msgs::Marker>("~/marker");

      prepareLineMarker(markerCenter, 1,
                        ignition::math::Color(1, 1, 0, 1)); // center: yellow
      prepareLineMarker(markerLeft, 2,
                        ignition::math::Color(0, 1, 0, 1)); // left:   green
      prepareLineMarker(markerRight, 3,
                        ignition::math::Color(1, 0, 0, 1)); // right:  red
    }

    // ===== CSV log =====
    if (logCsv) {
      system("mkdir -p logs >/dev/null 2>&1");
      csv.open(logPath, std::ios::out | std::ios::trunc);
      if (!csv) {
        gzerr << "[AOA] ⚠️ Cannot open CSV log path: " << logPath << "\n";
      } else {
        csv << "sim_time,dC_cm,dL_cm,dR_cm,state,vL,vR\n";
      }
    }

    // ===== Update Hook =====
    updateConn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AOAPlugin::OnUpdate, this));

    gzdbg << "[AOA] ✅ Plugin loaded (3-ray + wall-follow + dynamic colors + "
             "CSV log)\n";
  }

  ~AOAPlugin() override {
    if (csv.is_open())
      csv.close();
  }

private:
  // --------- helpers ----------
  template <typename T>
  static T getSdf(sdf::ElementPtr sdf, const std::string &key, const T &def) {
    return sdf->HasElement(key) ? sdf->Get<T>(key) : def;
  }

  // لون تدريجي حسب المسافة
  ignition::math::Color colorForDist(double dist_cm, double danger_cm,
                                     double max_cm) {
    double x =
        std::max(0.0, std::min(1.0, (dist_cm - danger_cm) /
                                        std::max(1.0, max_cm - danger_cm)));
    double r, g, b = 0.0;
    if (x < 0.5) {
      r = 1.0;
      g = 2.0 * x;
    } // red -> yellow
    else {
      r = 2.0 * (1.0 - x);
      g = 1.0;
    } // yellow -> green
    return ignition::math::Color(r, g, b, 1.0);
  }

  // Raycast بالنسبة لاتجاه الروبوت
  double castRayCm(double angleRad) {
    auto base = model->GetLink("base");
    if (!base)
      return 1e6;

    ignition::math::Pose3d pose = base->WorldPose();
    ignition::math::Vector3d start = pose.Pos();
    ignition::math::Vector3d forward(1, 0, 0);
    ignition::math::Quaterniond rotZ(0, 0, angleRad);
    ignition::math::Vector3d dir =
        pose.Rot().RotateVector(rotZ.RotateVector(forward));
    ignition::math::Vector3d end = start + dir * rayLenM;

    physics::RayShapePtr ray = boost::dynamic_pointer_cast<physics::RayShape>(
        physics->CreateShape("ray", physics::CollisionPtr()));
    ray->SetPoints(start, end);

    double dist;
    std::string entity;
    ray->GetIntersection(dist, entity);
    if (std::isfinite(dist))
      return dist * 100.0;
    return 1e6;
  }

  // إعداد Marker (IGN msgs)
  void prepareLineMarker(ignition::msgs::Marker &m, int id,
                         const ignition::math::Color &color) {
    m.set_ns("aoa_rays");
    m.set_id(id);
    m.set_action(ignition::msgs::Marker::ADD_MODIFY);
    m.set_type(ignition::msgs::Marker::LINE_LIST);
    msgs::Set(m.mutable_material()->mutable_ambient(), color);
    msgs::Set(m.mutable_material()->mutable_diffuse(), color);
    m.mutable_lifetime()->set_sec(0);
    m.mutable_scale()->set_x(0.02); // line width
  }

  // نشر الأشعة بالألوان والمسافات
  void updateRayMarkers(double dC_cm, double dL_cm, double dR_cm) {
    if (!vizRays || !markerPub)
      return;
    auto base = model->GetLink("base");
    if (!base)
      return;

    ignition::math::Pose3d pose = base->WorldPose();
    ignition::math::Vector3d start = pose.Pos();
    auto makeEnd = [&](double deg, double distCm) {
      double ang = deg * M_PI / 180.0;
      ignition::math::Vector3d fwd(1, 0, 0);
      ignition::math::Quaterniond rotZ(0, 0, ang);
      ignition::math::Vector3d dir =
          pose.Rot().RotateVector(rotZ.RotateVector(fwd));
      return start + dir * (distCm / 100.0);
    };

    const double max_cm = rayLenM * 100.0;

    auto publishRay = [&](ignition::msgs::Marker &m, double deg, double dist,
                          const ignition::math::Color &color) {
      ignition::msgs::Marker tmp = m; // copy base marker (id/ns/type/scale)
      tmp.mutable_point()->Clear();
      msgs::Set(tmp.add_point(), start);
      msgs::Set(tmp.add_point(), makeEnd(deg, std::min(dist, max_cm)));
      msgs::Set(tmp.mutable_material()->mutable_ambient(), color);
      msgs::Set(tmp.mutable_material()->mutable_diffuse(), color);
      markerPub->Publish(tmp);
    };

    publishRay(markerCenter, 0.0, dC_cm,
               colorForDist(dC_cm, safeDistCm, max_cm));
    publishRay(markerLeft, +leftDeg, dL_cm,
               colorForDist(dL_cm, safeDistCm, max_cm));
    publishRay(markerRight, +rightDeg, dR_cm,
               colorForDist(dR_cm, safeDistCm, max_cm));
  }

  // تحريك العجلات
  void setWheelSpeeds(double vLeft, double vRight) {
    double wL = vLeft / wheelRadius;
    double wR = vRight / wheelRadius;
    left->SetParam("fmax", 0, 30.0);
    right->SetParam("fmax", 0, 30.0);
    left->SetVelocity(0, wL);
    right->SetVelocity(0, wR);
    lastVL = vLeft;
    lastVR = vRight;
  }

  // حالات
  enum class State {
    FORWARD,
    AVOID_LEFT,
    AVOID_RIGHT,
    FOLLOW_LEFT,
    FOLLOW_RIGHT
  };

  void OnUpdate() {
    const double dC = castRayCm(0.0);
    const double dL = castRayCm(leftDeg * M_PI / 180.0);
    const double dR = castRayCm(rightDeg * M_PI / 180.0);

    updateRayMarkers(dC, dL, dR);

    const double danger = safeDistCm;
    const double sideDanger = danger * 0.8;
    const double clear = danger * 1.5;

    if (dC < danger) {
      state = (dL >= dR) ? State::AVOID_LEFT : State::AVOID_RIGHT;
    } else {
      if (dL < sideDanger && dR > dL + 5.0)
        state = State::FOLLOW_RIGHT;
      else if (dR < sideDanger && dL > dR + 5.0)
        state = State::FOLLOW_LEFT;
      else
        state = State::FORWARD;
    }

    if (state == State::FOLLOW_LEFT && dR > clear && dC > clear * 0.8)
      state = State::FORWARD;
    if (state == State::FOLLOW_RIGHT && dL > clear && dC > clear * 0.8)
      state = State::FORWARD;

    switch (state) {
    case State::FORWARD:
      setWheelSpeeds(fwdSpeed, fwdSpeed);
      break;
    case State::AVOID_LEFT:
      setWheelSpeeds(+turnSpeed, -turnSpeed);
      break;
    case State::AVOID_RIGHT:
      setWheelSpeeds(-turnSpeed, +turnSpeed);
      break;
    case State::FOLLOW_LEFT:
      setWheelSpeeds(+0.6 * turnSpeed, +0.2 * turnSpeed);
      break;
    case State::FOLLOW_RIGHT:
      setWheelSpeeds(+0.2 * turnSpeed, +0.6 * turnSpeed);
      break;
    }

    if (csv.is_open()) {
      double simt = world->SimTime().Double();
      csv << simt << "," << dC << "," << dL << "," << dR << ","
          << stateName(state) << "," << lastVL << "," << lastVR << "\n";
    }
  }

  static const char *stateName(State s) {
    switch (s) {
    case State::FORWARD:
      return "FORWARD";
    case State::AVOID_LEFT:
      return "AVOID_LEFT";
    case State::AVOID_RIGHT:
      return "AVOID_RIGHT";
    case State::FOLLOW_LEFT:
      return "FOLLOW_LEFT";
    case State::FOLLOW_RIGHT:
      return "FOLLOW_RIGHT";
    }
    return "UNKNOWN";
  }

  // ===== Members =====
  physics::WorldPtr world;
  physics::ModelPtr model;
  physics::PhysicsEnginePtr physics;
  physics::JointPtr left, right;
  event::ConnectionPtr updateConn;

  // Transport & Markers (IGN)
  transport::NodePtr node;
  transport::PublisherPtr markerPub;
  ignition::msgs::Marker markerCenter, markerLeft,
      markerRight; // ✅ أنواع ignition::msgs::Marker

  // Params
  std::string leftJointName, rightJointName;
  double wheelRadius{0.05}, wheelSep{0.24};
  double safeDistCm{30.0}, fwdSpeed{0.7}, turnSpeed{1.0};

  // Rays
  double rayLenM{2.0};
  double leftDeg{30.0}, rightDeg{-30.0};
  bool vizRays{true};

  // Logging
  bool logCsv{true};
  std::string logPath{"logs/aoa_log.csv"};
  std::ofstream csv;

  // State
  enum class StateEnum { dummy }; // لتفادي تحذيرات بعض المجمعات
  State state{State::FORWARD};
  double lastVL{0.0}, lastVR{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(AOAPlugin)
