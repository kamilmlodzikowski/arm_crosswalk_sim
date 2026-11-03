// Copyright 2026 Kamil Mlodzikowski
// SPDX-License-Identifier: Apache-2.0
//
// CrosswalkScenario - Gazebo Sim (Harmonic) world system that animates a
// signalised pedestrian crossing:
//   * traffic light cycle (vehicle + pedestrian signal heads),
//   * pedestrians (animated actors) crossing on green / walking along the
//     sidewalks, with simple social-force avoidance (also of the robot),
//   * cars (kinematic, IDM car-following) that stop on red, plus randomly
//     scheduled "red-light runners" that drive through while pedestrians
//     have GREEN,
//   * a referee that watches the robot (collisions, near misses, entering on
//     red, leaving the crosswalk, successful crossings).
//
// All pedestrians/cars/lamps are pre-created in the world SDF (object pools)
// and are only moved by this system, so nothing is spawned at run time.
// They have no collision geometry: the robot cannot be pushed by them, the
// referee reports contacts instead.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace crosswalk_sim
{
using gz::math::Pose3d;
using gz::math::Vector2d;
using gz::sim::ComponentState;
using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::kNullEntity;
namespace components = gz::sim::components;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kHiddenZ = -100.0;   // agents not in use are parked below ground
constexpr double kStep = 0.01;        // internal scenario time step [s]

double NormAngle(double a)
{
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}

// ---------------------------------------------------------------- geometry
struct Obb
{
  Vector2d c;
  double yaw{0};
  double hl{0};   // half length (along heading)
  double hw{0};   // half width
};

Vector2d Rot(const Vector2d &_v, double _a)
{
  const double c = std::cos(_a), s = std::sin(_a);
  return {c * _v.X() - s * _v.Y(), s * _v.X() + c * _v.Y()};
}

bool ObbOverlap(const Obb &_a, const Obb &_b)
{
  const std::array<Vector2d, 4> axes = {
    Rot({1, 0}, _a.yaw), Rot({0, 1}, _a.yaw),
    Rot({1, 0}, _b.yaw), Rot({0, 1}, _b.yaw)};
  const Vector2d d = _b.c - _a.c;
  for (const auto &ax : axes)
  {
    const double ra = _a.hl * std::abs(Rot({1, 0}, _a.yaw).Dot(ax)) +
                      _a.hw * std::abs(Rot({0, 1}, _a.yaw).Dot(ax));
    const double rb = _b.hl * std::abs(Rot({1, 0}, _b.yaw).Dot(ax)) +
                      _b.hw * std::abs(Rot({0, 1}, _b.yaw).Dot(ax));
    if (std::abs(d.Dot(ax)) > ra + rb)
      return false;
  }
  return true;
}

double PointObbDistance(const Vector2d &_p, const Obb &_b)
{
  const Vector2d l = Rot(_p - _b.c, -_b.yaw);
  const double dx = std::max(std::abs(l.X()) - _b.hl, 0.0);
  const double dy = std::max(std::abs(l.Y()) - _b.hw, 0.0);
  return std::hypot(dx, dy);
}

std::array<Vector2d, 4> Corners(const Obb &_b)
{
  std::array<Vector2d, 4> c;
  const int sx[4] = {1, 1, -1, -1};
  const int sy[4] = {1, -1, -1, 1};
  for (int i = 0; i < 4; ++i)
    c[i] = _b.c + Rot({sx[i] * _b.hl, sy[i] * _b.hw}, _b.yaw);
  return c;
}

double ObbDistance(const Obb &_a, const Obb &_b)
{
  if (ObbOverlap(_a, _b))
    return 0.0;
  double d = std::numeric_limits<double>::max();
  for (const auto &p : Corners(_a)) d = std::min(d, PointObbDistance(p, _b));
  for (const auto &p : Corners(_b)) d = std::min(d, PointObbDistance(p, _a));
  return d;
}

// -------------------------------------------------------------- json utils
std::string Num(double _v, int _prec = 3)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(_prec) << _v;
  return ss.str();
}

std::string Str(const std::string &_s)
{
  return "\"" + _s + "\"";
}

// ------------------------------------------------------------ sdf helpers
template <typename T>
T Param(const sdf::ElementConstPtr &_root, const std::string &_group,
        const std::string &_key, const T &_default)
{
  if (!_root)
    return _default;
  sdf::ElementConstPtr g = _root;
  if (!_group.empty())
  {
    if (!_root->HasElement(_group))
      return _default;
    g = _root->FindElement(_group);
  }
  if (!g || !g->HasElement(_key))
    return _default;
  return g->Get<T>(_key, _default).first;
}
}  // namespace

// =========================================================================
enum class Phase : int
{
  kCarGreen = 0,
  kCarYellow,
  kAllRed1,
  kPedGreen,
  kPedBlink,
  kAllRed2,
  kCarRedYellow,
  kCount
};

enum LampGroup : int
{
  kPedRed = 0,
  kPedGreen,
  kCarRed,
  kCarYellow,
  kCarGreenLamp,
  kGroupCount
};

struct Lamp
{
  std::string name;
  int group{0};
  Entity entity{kNullEntity};
  Pose3d onPose;
  bool on{true};
  bool known{false};
};

enum class PedStage
{
  kApproach,    // walking along the sidewalk towards the crosswalk
  kToCurb,      // walking to the waiting slot at the curb
  kWaiting,     // waiting for green
  kCrossing,    // on the crosswalk
  kLeaveTurn,   // stepping off the crosswalk onto the far sidewalk
  kLeave,       // walking away along the far sidewalk
  kWalk         // sidewalk walker (never crosses)
};

struct Pedestrian
{
  std::string name;
  Entity entity{kNullEntity};
  bool active{false};
  bool crosser{false};
  bool jaywalker{false};
  PedStage stage{PedStage::kWalk};
  int startSide{-1};      // -1 south sidewalk, +1 north
  int approachX{1};       // sign of x the pedestrian comes from
  int exitX{1};
  Vector2d pos, vel, target;
  double yaw{0};
  double speed{1.3};
  double crossX{0};
  int slotSide{-1}, slotRow{-1}, slotCol{-1};
  // animation
  bool walking{true};
  double animTime{0};
  bool animNameDirty{true};
  bool dirty{true};
  // referee
  bool contact{false};
};

struct Car
{
  std::string name;
  Entity entity{kNullEntity};
  double length{4.2};
  double width{1.8};
  double offset{0.0};   // OBB centre along heading, relative to model origin
  std::string type;
  bool active{false};
  int lane{0};
  double s{0};          // model origin along travel direction
  double v{0};
  double vDes{10};
  bool runner{false};
  bool runnerAnnounced{false};
  int yellow{0};        // 0 undecided, 1 go, -1 stop
  bool dirty{true};
  bool contact{false};
  bool nearMiss{false};

  double Front() const { return s + length / 2 + offset; }
  double Rear() const { return s - length / 2 + offset; }
};

struct PendingRunner
{
  bool pending{false};
  int lane{0};
  double crossTime{0};  // sim time at which the front reaches the crosswalk
  double speed{12};
};

// =========================================================================
class CrosswalkScenario
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    this->world = _entity;
    if (auto nameComp = _ecm.Component<components::Name>(_entity))
      this->worldName = nameComp->Data();
    this->LoadParams(_sdf);

    this->pubPedLight = this->node.Advertise<gz::msgs::StringMsg>(
      "/debug/crosswalk/ground_truth/pedestrian_light");
    this->pubCarLight = this->node.Advertise<gz::msgs::StringMsg>(
      "/debug/crosswalk/ground_truth/car_light");
    this->pubState = this->node.Advertise<gz::msgs::StringMsg>(
      "/debug/crosswalk/ground_truth/state");
    this->pubEvents = this->node.Advertise<gz::msgs::StringMsg>(
      "/debug/crosswalk/events");
    this->pubReferee = this->node.Advertise<gz::msgs::StringMsg>(
      "/debug/crosswalk/referee");
    this->node.Subscribe("/crosswalk/reset", &CrosswalkScenario::OnReset, this);

    gzmsg << "[CrosswalkScenario] " << this->peds.size() << " pedestrians, "
          << this->cars.size() << " cars, " << this->lamps.size()
          << " lamps in pool. Seed " << this->seedUsed << std::endl;
  }

  void Reset(const gz::sim::UpdateInfo &, EntityComponentManager &) override
  {
    this->initialized = false;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 EntityComponentManager &_ecm) override
  {
    const double simTime =
      std::chrono::duration<double>(_info.simTime).count();

    if (!this->ResolveEntities(_ecm))
      return;

    if (!this->initialized)
    {
      this->InitScenario(simTime);
      this->WriteEcm(_ecm, true);
      return;
    }
    if (_info.paused)
      return;

    this->UpdateRobot(_ecm);

    if (this->resetRequested.exchange(false))
      this->ResetTrial(_ecm);

    // fixed internal step, independent of the physics step size
    this->accum += simTime - this->lastSimTime;
    this->lastSimTime = simTime;
    if (this->accum < 0)
      this->accum = 0;
    int steps = 0;
    while (this->accum >= kStep && steps < 50)
    {
      this->now += kStep;
      this->Step(kStep, true);
      this->accum -= kStep;
      ++steps;
    }
    if (steps == 0)
      return;

    this->WriteEcm(_ecm, false);
    this->Publish();
  }

private:
  // ======================================================== parameters
  void LoadParams(const std::shared_ptr<const sdf::Element> &_sdf)
  {
    sdf::ElementConstPtr r = _sdf;
    auto &p = this->prm;
    p.seed = Param<int>(r, "", "seed", 0);
    p.warmup = Param<double>(r, "", "warmup", 30.0);

    p.laneWidth = Param<double>(r, "world", "lane_width", 3.5);
    p.sidewalkWidth = Param<double>(r, "world", "sidewalk_width", 4.0);
    p.crosswalkWidth = Param<double>(r, "world", "crosswalk_width", 4.0);
    p.stopLineOffset = Param<double>(r, "world", "stop_line_offset", 3.0);
    p.roadLength = Param<double>(r, "world", "road_length", 200.0);

    p.tCarGreen = Param<double>(r, "traffic_light", "car_green", 25.0);
    p.tCarYellow = Param<double>(r, "traffic_light", "car_yellow", 3.0);
    p.tAllRed = Param<double>(r, "traffic_light", "all_red", 2.0);
    p.tPedGreen = Param<double>(r, "traffic_light", "ped_green", 12.0);
    p.tPedBlink = Param<double>(r, "traffic_light", "ped_green_blink", 4.0);
    p.blinkPeriod = Param<double>(r, "traffic_light", "blink_period", 1.0);
    p.tCarRedYellow =
      Param<double>(r, "traffic_light", "car_red_yellow", 1.5);
    p.startPhase =
      Param<std::string>(r, "traffic_light", "start_phase", "random");

    p.pedCrossRate = Param<double>(r, "pedestrians", "crossing_rate", 8.0);
    p.pedWalkRate = Param<double>(r, "pedestrians", "sidewalk_rate", 6.0);
    p.pedSpeedMin = Param<double>(r, "pedestrians", "speed_min", 1.0);
    p.pedSpeedMax = Param<double>(r, "pedestrians", "speed_max", 1.6);
    p.pedRedProb =
      Param<double>(r, "pedestrians", "red_crossing_probability", 0.0);
    p.pedAvoidRobot = Param<bool>(r, "pedestrians", "avoid_robot", true);
    p.pedSpawnDist = Param<double>(r, "pedestrians", "spawn_distance", 35.0);
    p.animFactor = Param<double>(r, "pedestrians", "animation_factor", 4.588);

    p.carRate = Param<double>(r, "cars", "rate", 8.0);
    p.carSpeedMin = Param<double>(r, "cars", "speed_min", 8.0);
    p.carSpeedMax = Param<double>(r, "cars", "speed_max", 13.9);
    p.carYieldRobot = Param<bool>(r, "cars", "yield_to_robot", false);
    p.carYieldPed = Param<bool>(r, "cars", "yield_to_pedestrians", true);

    p.runnerProb = Param<double>(r, "red_runner", "probability", 0.3);
    const auto win = Param<gz::math::Vector2d>(
      r, "red_runner", "window", gz::math::Vector2d(0.0, 8.0));
    p.runnerWinMin = std::min(win.X(), win.Y());
    p.runnerWinMax = std::max(win.X(), win.Y());
    p.runnerSpeedMin = Param<double>(r, "red_runner", "speed_min", 10.0);
    p.runnerSpeedMax = Param<double>(r, "red_runner", "speed_max", 14.0);

    p.robotName = Param<std::string>(r, "robot", "name", "delivery_robot");
    const auto spawn = Param<gz::math::Vector3d>(
      r, "robot", "spawn", gz::math::Vector3d(0, -5, kPi / 2));
    p.robotSpawn = Pose3d(spawn.X(), spawn.Y(), 0.02, 0, 0, spawn.Z());
    const auto fp = Param<gz::math::Vector2d>(
      r, "robot", "footprint", gz::math::Vector2d(0.72, 0.62));
    p.robotLen = fp.X();
    p.robotWid = fp.Y();
    this->robotPose = p.robotSpawn;
    const auto goal = Param<gz::math::Vector2d>(
      r, "robot", "goal", gz::math::Vector2d(0, 6));
    p.goal = goal;

    p.nearMiss = Param<double>(r, "referee", "near_miss_distance", 1.0);
    p.pauseOnCollision =
      Param<bool>(r, "referee", "pause_on_collision", false);

    // sanity
    for (double *d : {&p.tCarGreen, &p.tCarYellow, &p.tAllRed, &p.tPedGreen,
                      &p.tPedBlink, &p.tCarRedYellow})
      *d = std::max(*d, 0.01);
    p.blinkPeriod = std::max(p.blinkPeriod, 0.1);
    p.pedSpeedMin = std::max(p.pedSpeedMin, 0.3);
    p.pedSpeedMax = std::max(p.pedSpeedMax, p.pedSpeedMin);
    p.carSpeedMin = std::max(p.carSpeedMin, 1.0);
    p.carSpeedMax = std::max(p.carSpeedMax, p.carSpeedMin);
    p.runnerSpeedMin = std::max(p.runnerSpeedMin, 1.0);
    p.runnerSpeedMax = std::max(p.runnerSpeedMax, p.runnerSpeedMin);

    this->roadHalf = p.laneWidth;          // one lane per direction
    this->cwHalf = p.crosswalkWidth / 2.0;
    this->stopLineS = -(this->cwHalf + p.stopLineOffset);
    this->spawnS = -(p.roadLength / 2.0 - 5.0);

    // pools
    for (auto el = r->FindElement("pedestrian"); el;
         el = el->GetNextElement("pedestrian"))
    {
      Pedestrian ped;
      ped.name = el->Get<std::string>("name");
      this->peds.push_back(ped);
    }
    for (auto el = r->FindElement("car"); el; el = el->GetNextElement("car"))
    {
      Car car;
      car.name = el->Get<std::string>("name");
      car.length = el->Get<double>("length");
      car.width = el->Get<double>("width");
      car.offset = el->Get<double>("offset");
      car.type = el->Get<std::string>("type");
      this->cars.push_back(car);
    }
    for (auto el = r->FindElement("lamp"); el; el = el->GetNextElement("lamp"))
    {
      Lamp lamp;
      lamp.name = el->Get<std::string>("name");
      const auto g = el->Get<std::string>("group");
      lamp.group = g == "ped_red" ? kPedRed :
                   g == "ped_green" ? kPedGreen :
                   g == "car_red" ? kCarRed :
                   g == "car_yellow" ? kCarYellow : kCarGreenLamp;
      this->lamps.push_back(lamp);
    }

    this->seedUsed = p.seed != 0 ?
      static_cast<unsigned>(p.seed) : std::random_device{}();
  }

  // ===================================================== entity lookup
  bool ResolveEntities(EntityComponentManager &_ecm)
  {
    if (this->resolved)
      return true;
    bool ok = true;
    for (auto &ped : this->peds)
    {
      if (ped.entity != kNullEntity) continue;
      // components::Actor holds data, so look up by name and check the type
      ped.entity = _ecm.EntityByComponents(components::Name(ped.name));
      if (ped.entity != kNullEntity &&
          !_ecm.Component<components::Actor>(ped.entity))
        ped.entity = kNullEntity;
      if (ped.entity == kNullEntity) { ok = false; continue; }
      // trajectory pose is the world pose, Pose holds only the height offset
      auto poseComp = _ecm.Component<components::Pose>(ped.entity);
      const double z = poseComp ? poseComp->Data().Pos().Z() : 1.0;
      this->actorZ = z > 0.1 ? z : 1.0;
      if (poseComp)
        *poseComp = components::Pose(Pose3d(0, 0, this->actorZ, 0, 0, 0));
      else
        _ecm.CreateComponent(ped.entity,
          components::Pose(Pose3d(0, 0, this->actorZ, 0, 0, 0)));
      if (!_ecm.Component<components::TrajectoryPose>(ped.entity))
        _ecm.CreateComponent(ped.entity, components::TrajectoryPose(
          Pose3d(0, 0, kHiddenZ, 0, 0, 0)));
      if (!_ecm.Component<components::AnimationName>(ped.entity))
        _ecm.CreateComponent(ped.entity, components::AnimationName("walk"));
      if (!_ecm.Component<components::AnimationTime>(ped.entity))
        _ecm.CreateComponent(ped.entity, components::AnimationTime());
    }
    for (auto &car : this->cars)
    {
      if (car.entity != kNullEntity) continue;
      car.entity = _ecm.EntityByComponents(components::Model(),
                                           components::Name(car.name));
      if (car.entity == kNullEntity) ok = false;
    }
    for (auto &lamp : this->lamps)
    {
      if (lamp.entity != kNullEntity) continue;
      lamp.entity = _ecm.EntityByComponents(components::Model(),
                                            components::Name(lamp.name));
      if (lamp.entity == kNullEntity) { ok = false; continue; }
      if (auto pc = _ecm.Component<components::Pose>(lamp.entity))
        lamp.onPose = pc->Data();
    }
    if (!ok)
    {
      if (++this->resolveAttempts == 2000)
        gzerr << "[CrosswalkScenario] some pool entities were not found in "
              << "the world - check the generated SDF." << std::endl;
      return false;
    }
    this->resolved = true;
    return true;
  }

  // ======================================================= init / reset
  void InitScenario(double _simTime)
  {
    this->rng.seed(this->seedUsed);
    this->now = 0.0;
    this->lastSimTime = _simTime;
    this->accum = 0.0;
    for (auto &ped : this->peds) { ped.active = false; ped.dirty = true; }
    for (auto &car : this->cars) { car.active = false; car.dirty = true; }
    for (auto &row : this->slots) for (auto &r : row) r.fill(false);
    this->runner = PendingRunner();
    this->laneBlocked = {false, false};
    this->runnerDecided = false;

    // traffic light start
    const auto &p = this->prm;
    if (p.startPhase == "ped_green")
      this->SetPhase(Phase::kPedGreen, 0.0);
    else if (p.startPhase == "car_green")
      this->SetPhase(Phase::kCarGreen, 0.0);
    else
    {
      double t = this->Uniform(0.0, this->CycleLength());
      int i = 0;
      while (t >= this->Duration(static_cast<Phase>(i)))
      {
        t -= this->Duration(static_cast<Phase>(i));
        i = (i + 1) % static_cast<int>(Phase::kCount);
      }
      this->SetPhase(static_cast<Phase>(i), t);
    }

    this->nextCrosser = this->Exp(p.pedCrossRate);
    this->nextWalker = this->Exp(p.pedWalkRate);
    this->nextCar = {this->Exp(p.carRate), this->Exp(p.carRate)};

    // pre-simulate so that the street is already busy at t = 0
    const double warmup = std::max(0.0, p.warmup);
    const double warmStep = 0.05;
    for (double t = 0; t < warmup; t += warmStep)
    {
      this->now += warmStep;
      this->Step(warmStep, false);
    }
    // shift scenario clock so that t = 0 is "now"
    const double shift = this->now;
    this->now = 0.0;
    this->nextCrosser -= shift;
    this->nextWalker -= shift;
    for (auto &n : this->nextCar) n -= shift;
    if (this->runner.pending) this->runner.crossTime -= shift;

    for (auto &ped : this->peds) ped.dirty = true;
    for (auto &car : this->cars) car.dirty = true;
    for (auto &lamp : this->lamps) lamp.known = false;
    this->ResetReferee();
    this->initialized = true;
    this->lastPedLight.clear();
    this->lastCarLight.clear();
    this->Event("{\"type\":\"scenario_started\",\"seed\":" +
                std::to_string(this->seedUsed) + "}");
  }

  void ResetReferee()
  {
    this->attemptActive = false;
    this->attempts = 0;
    this->successes = 0;
    this->collisionsTotal = 0;
    this->lastResult = "NONE";
    this->attemptViolations.clear();
    this->attemptCollisions = 0;
    this->goalReported = false;
    for (auto &c : this->cars) { c.contact = false; c.nearMiss = false; }
    for (auto &pd : this->peds) pd.contact = false;
  }

  void ResetTrial(EntityComponentManager &_ecm)
  {
    if (this->robot != kNullEntity)
    {
      gz::sim::Model model(this->robot);
      model.SetWorldPoseCmd(_ecm, this->prm.robotSpawn);
      this->robotPose = this->prm.robotSpawn;
    }
    const Vector2d sp(this->prm.robotSpawn.Pos().X(),
                      this->prm.robotSpawn.Pos().Y());
    this->ClearPedestriansAround(sp, 1.2);
    this->ResetReferee();
    this->robotZone = this->ZoneOf(sp.Y());
    this->Event("{\"type\":\"reset\"}");
  }

  void ClearPedestriansAround(const Vector2d &_p, double _r)
  {
    for (auto &ped : this->peds)
      if (ped.active && ped.pos.Distance(_p) < _r)
        this->Deactivate(ped);
  }

  void OnReset(const gz::msgs::Empty &)
  {
    this->resetRequested = true;
  }

  // ===================================================== random helpers
  double Uniform(double _a, double _b)
  {
    if (_b <= _a) return _a;
    return std::uniform_real_distribution<double>(_a, _b)(this->rng);
  }
  bool Chance(double _p)
  {
    return this->Uniform(0.0, 1.0) < _p;
  }
  // exponential inter-arrival time for a rate given per minute
  double Exp(double _ratePerMin)
  {
    if (_ratePerMin <= 1e-6)
      return std::numeric_limits<double>::infinity();
    return std::exponential_distribution<double>(_ratePerMin / 60.0)(this->rng);
  }

  // ===================================================== traffic light
  double Duration(Phase _p) const
  {
    const auto &p = this->prm;
    switch (_p)
    {
      case Phase::kCarGreen: return p.tCarGreen;
      case Phase::kCarYellow: return p.tCarYellow;
      case Phase::kAllRed1: return p.tAllRed;
      case Phase::kPedGreen: return p.tPedGreen;
      case Phase::kPedBlink: return p.tPedBlink;
      case Phase::kAllRed2: return p.tAllRed;
      case Phase::kCarRedYellow: return p.tCarRedYellow;
      default: return 1.0;
    }
  }

  double CycleLength() const
  {
    double t = 0;
    for (int i = 0; i < static_cast<int>(Phase::kCount); ++i)
      t += this->Duration(static_cast<Phase>(i));
    return t;
  }

  void SetPhase(Phase _p, double _t)
  {
    this->phase = _p;
    this->phaseT = _t;
    this->OnPhaseEnter(_p);
  }

  void AdvanceLights(double _dt)
  {
    this->phaseT += _dt;
    while (this->phaseT >= this->Duration(this->phase))
    {
      this->phaseT -= this->Duration(this->phase);
      const auto next = static_cast<Phase>(
        (static_cast<int>(this->phase) + 1) % static_cast<int>(Phase::kCount));
      this->phase = next;
      this->OnPhaseEnter(next);
    }
  }

  void OnPhaseEnter(Phase _p)
  {
    if (_p == Phase::kCarGreen)
    {
      for (auto &c : this->cars) c.yellow = 0;
      this->runnerDecided = false;
    }
    else if (_p == Phase::kCarYellow)
    {
      for (auto &c : this->cars)
      {
        if (!c.active || c.runner) continue;
        const double dist = this->stopLineS - c.Front();
        const double need = c.v * c.v / (2.0 * 3.0);
        c.yellow = (dist > need) ? -1 : 1;
      }
    }
  }

  // time until the next pedestrian GREEN onset (0 if it is green now)
  double TimeToPedGreen() const
  {
    if (this->phase == Phase::kPedGreen) return 0.0;
    double t = this->Duration(this->phase) - this->phaseT;
    int i = (static_cast<int>(this->phase) + 1) % static_cast<int>(Phase::kCount);
    while (static_cast<Phase>(i) != Phase::kPedGreen)
    {
      t += this->Duration(static_cast<Phase>(i));
      i = (i + 1) % static_cast<int>(Phase::kCount);
    }
    return t;
  }

  std::string PedLight() const
  {
    if (this->phase == Phase::kPedGreen) return "GREEN";
    if (this->phase == Phase::kPedBlink) return "GREEN_BLINKING";
    return "RED";
  }

  std::string CarLight() const
  {
    switch (this->phase)
    {
      case Phase::kCarGreen: return "GREEN";
      case Phase::kCarYellow: return "YELLOW";
      case Phase::kCarRedYellow: return "RED_YELLOW";
      default: return "RED";
    }
  }

  // seconds until the pedestrian light changes its state
  double PedLightRemaining() const
  {
    const std::string cur = this->PedLight();
    double t = this->Duration(this->phase) - this->phaseT;
    int i = (static_cast<int>(this->phase) + 1) % static_cast<int>(Phase::kCount);
    for (int k = 0; k < static_cast<int>(Phase::kCount); ++k)
    {
      const auto ph = static_cast<Phase>(i);
      const std::string st = ph == Phase::kPedGreen ? "GREEN" :
                             ph == Phase::kPedBlink ? "GREEN_BLINKING" : "RED";
      if (st != cur) break;
      t += this->Duration(ph);
      i = (i + 1) % static_cast<int>(Phase::kCount);
    }
    return t;
  }

  bool LampOn(int _group) const
  {
    switch (_group)
    {
      case kPedRed:
        return this->PedLight() == "RED";
      case kPedGreen:
        if (this->phase == Phase::kPedGreen) return true;
        if (this->phase == Phase::kPedBlink)
          return std::fmod(this->phaseT, this->prm.blinkPeriod) <
                 this->prm.blinkPeriod / 2.0;
        return false;
      case kCarRed:
        return this->phase != Phase::kCarGreen &&
               this->phase != Phase::kCarYellow;
      case kCarYellow:
        return this->phase == Phase::kCarYellow ||
               this->phase == Phase::kCarRedYellow;
      case kCarGreenLamp:
        return this->phase == Phase::kCarGreen;
      default:
        return false;
    }
  }

  // ====================================================== main step
  void Step(double _dt, bool _live)
  {
    this->AdvanceLights(_dt);
    this->ScheduleRunner(_live);
    this->SpawnCars();
    this->SpawnPedestrians();
    this->UpdateCars(_dt, _live);
    this->UpdatePedestrians(_dt);
    if (_live)
      this->UpdateReferee();
  }

  // ====================================================== red runners
  void ScheduleRunner(bool _live)
  {
    const auto &p = this->prm;
    // decide well before the cars get yellow so that the runner's lane can
    // be kept free of cars that would stop at the red light
    const double leadClear = 14.0;
    if (!this->runnerDecided && this->phase == Phase::kCarGreen &&
        this->phaseT >= std::max(0.0, p.tCarGreen - leadClear))
    {
      this->runnerDecided = true;
      if (!this->runner.pending && this->Chance(p.runnerProb))
      {
        this->runner.pending = true;
        this->runner.lane = this->Chance(0.5) ? 0 : 1;
        this->runner.speed = this->Uniform(p.runnerSpeedMin, p.runnerSpeedMax);
        this->runner.crossTime = this->now + this->TimeToPedGreen() +
          this->Uniform(p.runnerWinMin, p.runnerWinMax);
        this->laneBlocked[this->runner.lane] = true;
        if (_live)
          this->Event("{\"type\":\"red_runner_scheduled\",\"lane\":" +
            std::to_string(this->runner.lane) + ",\"direction\":" +
            Str(this->runner.lane == 0 ? "+x" : "-x") +
            ",\"crosswalk_eta\":" + Num(this->runner.crossTime - this->now) +
            ",\"after_ped_green\":" +
            Num(this->runner.crossTime - this->now - this->TimeToPedGreen()) +
            ",\"speed\":" + Num(this->runner.speed) + "}");
      }
    }

    if (!this->runner.pending)
      return;

    // spawn so that the front bumper reaches the crosswalk at crossTime
    Car *free = this->FreeCar();
    if (!free)
      return;
    const double frontAtCross = -this->cwHalf;
    const double frontNow = frontAtCross -
      this->runner.speed * (this->runner.crossTime - this->now);
    const double front0 = this->spawnS + free->length / 2 + free->offset;
    if (frontNow < front0)
      return;  // not yet
    // if we are late, spawn closer (but never on the crosswalk itself)
    this->ActivateCar(*free, this->runner.lane,
                      std::min(frontNow, frontAtCross - 5.0) -
                        free->length / 2 - free->offset,
                      this->runner.speed, true);
    this->runner.pending = false;
    this->laneBlocked[this->runner.lane] = false;
  }

  // ============================================================ cars
  double LaneY(int _lane) const
  {
    return _lane == 0 ? -this->prm.laneWidth / 2 : this->prm.laneWidth / 2;
  }
  int LaneDir(int _lane) const { return _lane == 0 ? 1 : -1; }

  Car *FreeCar()
  {
    std::vector<Car *> free;
    for (auto &c : this->cars)
      if (!c.active && c.entity != kNullEntity) free.push_back(&c);
    if (free.empty()) return nullptr;
    std::uniform_int_distribution<size_t> d(0, free.size() - 1);
    return free[d(this->rng)];
  }

  void ActivateCar(Car &_c, int _lane, double _s, double _v, bool _runner)
  {
    _c.active = true;
    _c.lane = _lane;
    _c.s = _s;
    _c.vDes = _v;
    _c.v = _v;
    _c.runner = _runner;
    _c.runnerAnnounced = false;
    _c.yellow = (this->phase == Phase::kCarGreen) ? 0 : -1;
    _c.contact = false;
    _c.nearMiss = false;
    _c.dirty = true;
  }

  void SpawnCars()
  {
    const auto &p = this->prm;
    for (int lane = 0; lane < 2; ++lane)
    {
      if (this->now < this->nextCar[lane]) continue;
      if (this->laneBlocked[lane])
      {
        this->nextCar[lane] = this->now + this->Exp(p.carRate);
        continue;
      }
      Car *c = this->FreeCar();
      if (!c) { this->nextCar[lane] = this->now + 1.0; continue; }
      const double vDes = this->Uniform(p.carSpeedMin, p.carSpeedMax);
      const double front = this->spawnS + c->length / 2 + c->offset;
      // gap to the last car in this lane
      double gap = std::numeric_limits<double>::max();
      double leaderV = vDes;
      for (const auto &o : this->cars)
      {
        if (!o.active || o.lane != lane) continue;
        const double g = o.Rear() - front;
        if (g > -1.0 && g < gap) { gap = g; leaderV = o.v; }
        if (g <= -1.0 && o.Front() > this->spawnS - 10) gap = -1;
      }
      if (gap < 12.0 + 1.0 * vDes)
      {
        this->nextCar[lane] = this->now + 0.5;
        continue;
      }
      this->ActivateCar(*c, lane, this->spawnS, std::min(vDes, leaderV + 2.0),
                        false);
      c->vDes = vDes;
      this->nextCar[lane] = this->now + this->Exp(p.carRate);
    }
  }

  void UpdateCars(double _dt, bool _live)
  {
    const auto &p = this->prm;
    const double aMax = 2.0, bComf = 3.0, bMax = 8.0, T = 1.2, s0 = 2.0;
    const std::string carLight = this->CarLight();

    for (auto &c : this->cars)
    {
      if (!c.active) continue;
      const int dir = this->LaneDir(c.lane);
      const double laneY = this->LaneY(c.lane);
      const double front = c.Front();

      double gap = std::numeric_limits<double>::max();
      double vl = 0.0;
      auto consider = [&](double _g, double _v)
      {
        if (_g < gap) { gap = _g; vl = _v; }
      };

      // leader car
      for (const auto &o : this->cars)
      {
        if (&o == &c || !o.active || o.lane != c.lane) continue;
        if (o.s <= c.s) continue;
        consider(o.Rear() - front, o.v);
      }
      // stop line
      bool mustStop = false;
      if (!c.runner)
      {
        if (carLight == "RED" || carLight == "RED_YELLOW")
          mustStop = true;
        else if (carLight == "YELLOW")
          mustStop = (c.yellow != 1);
      }
      if (mustStop && front < this->stopLineS + 0.5)
        consider((this->stopLineS - 0.3 + s0) - front, 0.0);

      // pedestrians on the carriageway
      if (p.carYieldPed)
      {
        // a red-light runner is an inattentive driver: it only reacts to
        // people directly in its path and only when they are close
        const double band = c.runner ? c.width / 2 + 0.5 : p.laneWidth / 2 + 1.0;
        const double reach = c.runner ?
          c.v * 0.8 + c.v * c.v / (2.0 * bMax) + 4.0 : 1e9;
        for (const auto &pd : this->peds)
        {
          if (!pd.active || pd.stage != PedStage::kCrossing) continue;
          if (std::abs(pd.pos.Y()) > this->roadHalf + 1.0) continue;
          if (std::abs(pd.pos.Y() - laneY) > band) continue;
          const double sp = dir * pd.pos.X();
          if (sp + 0.3 < front - 0.5 || sp - front > reach) continue;
          consider(sp - 0.4 - front + (s0 - 1.5), 0.0);
        }
      }
      // robot
      if (p.carYieldRobot && this->robot != kNullEntity)
      {
        const double ry = this->robotPose.Pos().Y();
        if (std::abs(ry) < this->roadHalf + 0.3 &&
            std::abs(ry - laneY) < p.laneWidth / 2 + this->prm.robotWid)
        {
          const double sr = dir * this->robotPose.Pos().X();
          if (sr + 0.5 > front - 0.5)
            consider(sr - p.robotLen / 2 - front + (s0 - 1.0), 0.0);
        }
      }

      // Intelligent Driver Model
      const double v = c.v;
      double a = aMax * (1.0 - std::pow(v / std::max(c.vDes, 0.1), 4));
      if (gap < 500.0)
      {
        const double sStar =
          s0 + std::max(0.0, v * T + v * (v - vl) / (2 * std::sqrt(aMax * bComf)));
        a -= aMax * std::pow(sStar / std::max(gap, 0.05), 2);
      }
      a = std::max(a, -bMax);
      c.v = std::max(0.0, v + a * _dt);
      c.s += c.v * _dt;
      c.dirty = true;

      if (c.runner && _live && !c.runnerAnnounced && c.Front() >= -this->cwHalf)
      {
        c.runnerAnnounced = true;
        this->Event("{\"type\":\"red_runner_at_crosswalk\",\"car\":" +
          Str(c.name) + ",\"pedestrian_light\":" + Str(this->PedLight()) +
          ",\"speed\":" + Num(c.v) + "}");
      }

      if (c.Rear() > p.roadLength / 2 - 5.0)
      {
        c.active = false;
        c.dirty = true;
      }
    }
  }

  Obb CarObb(const Car &_c) const
  {
    const int dir = this->LaneDir(_c.lane);
    Obb b;
    b.c = Vector2d(dir * (_c.s + _c.offset), this->LaneY(_c.lane));
    b.yaw = dir > 0 ? 0.0 : kPi;
    b.hl = _c.length / 2;
    b.hw = _c.width / 2;
    return b;
  }

  // ===================================================== pedestrians
  static constexpr int kRows = 3;
  static constexpr int kCols = 3;

  Vector2d SlotPos(int _side, int _row, int _col, int _crossDir) const
  {
    // crossers keep right: walking +y -> x > 0, walking -y -> x < 0
    const double x0 = 0.4, x1 = std::max(this->cwHalf - 0.3, x0 + 0.65 * (kCols - 1));
    const double x = x0 + _col * (x1 - x0) / (kCols - 1);
    const double y = _side * (this->roadHalf + 0.45 + _row * 0.7);
    return {_crossDir > 0 ? x : -x, y};
  }

  bool AssignSlot(Pedestrian &_p)
  {
    const int side = _p.startSide;
    const int si = side < 0 ? 0 : 1;
    const int crossDir = -side;
    std::vector<int> cols = {0, 1, 2};
    for (int row = 0; row < kRows; ++row)
    {
      std::shuffle(cols.begin(), cols.end(), this->rng);
      for (int col : cols)
      {
        if (this->slots[si][row][col]) continue;
        const Vector2d sp = this->SlotPos(side, row, col, crossDir);
        if (this->RobotBlocksSlot(sp))
          continue;
        this->slots[si][row][col] = true;
        _p.slotSide = si;
        _p.slotRow = row;
        _p.slotCol = col;
        _p.target = sp;
        _p.crossX = sp.X();
        return true;
      }
    }
    // no free slot: wait a bit further back
    _p.slotRow = -1;
    const Vector2d sp = this->SlotPos(side, kRows, this->Chance(0.5) ? 0 : 2,
                                      crossDir);
    _p.target = sp + Vector2d(this->Uniform(-0.2, 0.2), 0);
    _p.crossX = _p.target.X();
    return false;
  }

  bool RobotBlocksSlot(const Vector2d &_sp) const
  {
    const Vector2d r(this->robotPose.Pos().X(), this->robotPose.Pos().Y());
    if (_sp.Distance(r) < 1.3)
      return true;
    const Vector2d l = Rot(_sp - r, -this->robotPose.Rot().Yaw());
    return l.X() > 0 && l.X() < 3.0 && std::abs(l.Y()) < 0.8;
  }

  void ReleaseSlot(Pedestrian &_p)
  {
    if (_p.slotRow >= 0)
      this->slots[_p.slotSide][_p.slotRow][_p.slotCol] = false;
    _p.slotRow = -1;
  }

  // random lateral position on a sidewalk, keeping right
  double SidewalkLaneY(int _side, int _dirX)
  {
    const double inner = this->roadHalf;
    const double w = this->prm.sidewalkWidth;
    const bool outer = (_side * _dirX) < 0;
    double lo, hi;
    if (outer) { lo = inner + w / 2 + 0.25; hi = inner + w - 0.45; }
    else { lo = inner + 0.55; hi = inner + w / 2 - 0.25; }
    if (hi < lo) hi = lo;
    return _side * this->Uniform(lo, hi);
  }

  Pedestrian *FreePed()
  {
    std::vector<Pedestrian *> free;
    for (auto &pd : this->peds)
      if (!pd.active && pd.entity != kNullEntity) free.push_back(&pd);
    if (free.empty()) return nullptr;
    std::uniform_int_distribution<size_t> d(0, free.size() - 1);
    return free[d(this->rng)];
  }

  bool SpawnSpotFree(const Vector2d &_p) const
  {
    for (const auto &o : this->peds)
      if (o.active && o.pos.Distance(_p) < 1.2) return false;
    return true;
  }

  void SpawnPedestrians()
  {
    const auto &p = this->prm;
    if (this->now >= this->nextCrosser)
    {
      Pedestrian *pd = this->FreePed();
      if (pd)
      {
        const int side = this->Chance(0.5) ? -1 : 1;
        const int ax = this->Chance(0.5) ? -1 : 1;
        const Vector2d start(ax * p.pedSpawnDist,
                             this->SidewalkLaneY(side, -ax));
        if (this->SpawnSpotFree(start))
        {
          this->ActivatePed(*pd, start, true, side, ax);
          this->nextCrosser = this->now + this->Exp(p.pedCrossRate);
        }
        else
          this->nextCrosser = this->now + 0.5;
      }
      else
        this->nextCrosser = this->now + 1.0;
    }
    if (this->now >= this->nextWalker)
    {
      Pedestrian *pd = this->FreePed();
      if (pd)
      {
        const int side = this->Chance(0.5) ? -1 : 1;
        const int ax = this->Chance(0.5) ? -1 : 1;
        const Vector2d start(ax * p.pedSpawnDist,
                             this->SidewalkLaneY(side, -ax));
        if (this->SpawnSpotFree(start))
        {
          this->ActivatePed(*pd, start, false, side, ax);
          this->nextWalker = this->now + this->Exp(p.pedWalkRate);
        }
        else
          this->nextWalker = this->now + 0.5;
      }
      else
        this->nextWalker = this->now + 1.0;
    }
  }

  void ActivatePed(Pedestrian &_p, const Vector2d &_start, bool _crosser,
                   int _side, int _ax)
  {
    const auto &p = this->prm;
    _p.active = true;
    _p.crosser = _crosser;
    _p.jaywalker = _crosser && this->Chance(p.pedRedProb);
    _p.startSide = _side;
    _p.approachX = _ax;
    _p.pos = _start;
    _p.speed = this->Uniform(p.pedSpeedMin, p.pedSpeedMax);
    _p.vel = Vector2d(-_ax * _p.speed, 0);
    _p.yaw = _ax > 0 ? kPi : 0.0;
    _p.slotRow = -1;
    _p.contact = false;
    _p.walking = true;
    _p.animNameDirty = true;
    _p.animTime = this->Uniform(0.0, 5.0);
    _p.dirty = true;
    if (_crosser)
    {
      _p.stage = PedStage::kApproach;
      _p.target = Vector2d(_ax * (this->cwHalf + 2.5), _start.Y());
    }
    else
    {
      _p.stage = PedStage::kWalk;
      _p.target = Vector2d(-_ax * p.pedSpawnDist, _start.Y());
    }
  }

  void Deactivate(Pedestrian &_p)
  {
    this->ReleaseSlot(_p);
    _p.active = false;
    _p.dirty = true;
  }

  bool GapForJaywalker() const
  {
    for (const auto &c : this->cars)
    {
      if (!c.active) continue;
      const double front = c.Front();
      if (front > -this->cwHalf - 1.0 && c.Rear() < this->cwHalf + 1.0)
        return false;               // car on / at the crosswalk
      if (front < -this->cwHalf)
      {
        const double tta = (-this->cwHalf - front) / std::max(c.v, 0.5);
        if (tta < 6.0) return false;
      }
    }
    return true;
  }

  bool CrosswalkOccupied() const
  {
    for (const auto &c : this->cars)
      if (c.active && c.Front() > -this->cwHalf - 0.3 &&
          c.Rear() < this->cwHalf + 0.3)
        return true;
    return false;
  }

  bool CanCross(const Pedestrian &_p) const
  {
    if (this->phase == Phase::kPedGreen)
      return !this->CrosswalkOccupied();
    return _p.jaywalker && this->GapForJaywalker();
  }

  void UpdatePedestrians(double _dt)
  {
    const auto &p = this->prm;
    const Vector2d rpos(this->robotPose.Pos().X(), this->robotPose.Pos().Y());
    const double robotR =
      0.5 * std::hypot(p.robotLen, p.robotWid) * 0.85;
    const double pedR = 0.27;

    for (auto &pd : this->peds)
    {
      if (!pd.active) continue;

      // ---- stage logic
      const double dist = pd.pos.Distance(pd.target);
      switch (pd.stage)
      {
        case PedStage::kApproach:
          if (std::abs(pd.pos.X()) < this->cwHalf + 3.0 || dist < 0.4)
          {
            this->AssignSlot(pd);
            pd.stage = PedStage::kToCurb;
          }
          break;
        case PedStage::kToCurb:
          if (this->CanCross(pd) && std::abs(pd.pos.X() - pd.crossX) < 1.0)
          {
            this->StartCrossing(pd);
          }
          else if (dist < 0.12)
          {
            pd.stage = PedStage::kWaiting;
            pd.vel = Vector2d::Zero;
          }
          break;
        case PedStage::kWaiting:
          if (this->CanCross(pd))
            this->StartCrossing(pd);
          break;
        case PedStage::kCrossing:
          if (dist < 0.4)
          {
            pd.stage = PedStage::kLeaveTurn;
            pd.exitX = this->Chance(0.5) ? -1 : 1;
            const int side = -pd.startSide;
            pd.target = Vector2d(pd.exitX * (this->cwHalf + 1.5),
                                 this->SidewalkLaneY(side, pd.exitX));
          }
          break;
        case PedStage::kLeaveTurn:
          if (dist < 0.5)
          {
            pd.stage = PedStage::kLeave;
            pd.target = Vector2d(pd.exitX * p.pedSpawnDist, pd.target.Y());
          }
          break;
        case PedStage::kLeave:
        case PedStage::kWalk:
          if (dist < 0.5)
          {
            this->Deactivate(pd);
            continue;
          }
          break;
      }

      // ---- motion (social force)
      if (pd.stage == PedStage::kWaiting)
      {
        pd.vel = Vector2d::Zero;
        const double want = pd.startSide < 0 ? kPi / 2 : -kPi / 2;
        pd.yaw += std::clamp(NormAngle(want - pd.yaw), -3.0 * _dt, 3.0 * _dt);
      }
      else
      {
        Vector2d e = pd.target - pd.pos;
        const double d = e.Length();
        if (d > 1e-6) e /= d;
        double vmag = pd.speed;
        if (pd.stage == PedStage::kCrossing && this->LaneAheadDangerous(pd))
          vmag = 0.0;
        // close to the waiting spot: people stand close to each other
        const bool arriving = pd.stage == PedStage::kToCurb && d < 1.5;
        if (pd.stage == PedStage::kToCurb && !this->CanCross(pd))
          vmag *= std::clamp(d / 1.2, 0.25, 1.0);
        Vector2d force = (e * vmag - pd.vel) / 0.5;

        const Vector2d heading = pd.vel.Length() > 0.1 ?
          pd.vel / pd.vel.Length() : e;
        auto repulse = [&](const Vector2d &_other, double _r, double _a,
                           double _b, bool _sidestep)
        {
          Vector2d rel = pd.pos - _other;
          double dd = rel.Length();
          if (dd > 4.0) return;
          if (dd < 1e-3) { rel = Vector2d(0.01, 0); dd = 0.01; }
          const Vector2d n = rel / dd;
          const double cosPhi = -n.Dot(heading);
          const double w = 0.35 + 0.65 * (1.0 + cosPhi) / 2.0;
          const double mag = _a * std::exp((_r - dd) / _b) * w;
          force += n * mag;
          if (_sidestep && cosPhi > 0.6)
          {
            // keep right to resolve head-on encounters
            const Vector2d right(heading.Y(), -heading.X());
            force += right * (0.8 * mag);
          }
        };
        for (const auto &o : this->peds)
        {
          if (&o == &pd || !o.active) continue;
          if (arriving && d < 0.6) continue;   // final approach to the slot
          repulse(o.pos, 2 * pedR, arriving ? 0.6 : 1.8, 0.3,
                  o.stage != PedStage::kWaiting);
        }
        if (p.pedAvoidRobot)
          repulse(rpos, robotR + pedR + 0.35, 4.0, 0.35, true);

        pd.vel += force * _dt;
        const double vlen = pd.vel.Length();
        const double vcap = 1.3 * pd.speed;
        if (vlen > vcap) pd.vel *= vcap / vlen;
        pd.pos += pd.vel * _dt;

        // stay on the walkable area
        if (pd.stage == PedStage::kWalk || pd.stage == PedStage::kLeave ||
            pd.stage == PedStage::kApproach || pd.stage == PedStage::kToCurb)
        {
          const int side = pd.pos.Y() < 0 ? -1 : 1;
          const double lo = this->roadHalf + 0.35;
          const double hi = this->roadHalf + p.sidewalkWidth - 0.3;
          pd.pos.Y(side * std::clamp(std::abs(pd.pos.Y()), lo, hi));
        }
        if (pd.stage == PedStage::kCrossing)
          pd.pos.X(std::clamp(pd.pos.X(), -this->cwHalf - 0.2,
                              this->cwHalf + 0.2));

        if (pd.vel.Length() > 0.2)
        {
          const double want = std::atan2(pd.vel.Y(), pd.vel.X());
          pd.yaw += std::clamp(NormAngle(want - pd.yaw), -4.0 * _dt, 4.0 * _dt);
          pd.yaw = NormAngle(pd.yaw);
        }
      }

      // ---- animation
      const double spd = pd.vel.Length();
      const bool walk = pd.walking ? spd > 0.12 : spd > 0.3;
      if (walk != pd.walking)
      {
        pd.walking = walk;
        pd.animNameDirty = true;
      }
      pd.animTime += pd.walking ? spd * _dt * p.animFactor : _dt;
      pd.dirty = true;
    }
  }

  // true if the crossing pedestrian is about to step into a lane in which a
  // car is approaching fast (e.g. a red-light runner) or passing right now
  bool LaneAheadDangerous(const Pedestrian &_p) const
  {
    const int crossDir = -_p.startSide;
    const double y = _p.pos.Y();
    for (int lane = 0; lane < 2; ++lane)
    {
      const double lo = lane == 0 ? -this->roadHalf : 0.0;
      const double hi = lane == 0 ? 0.0 : this->roadHalf;
      if (y > lo - 0.1 && y < hi + 0.1) continue;      // already inside
      const double edge = crossDir > 0 ? lo : hi;
      const double toEdge = (edge - y) * crossDir;
      if (toEdge < 0.0 || toEdge > 0.9) continue;      // not entering it now
      for (const auto &c : this->cars)
      {
        if (!c.active || c.lane != lane) continue;
        const double front = c.Front();
        if (front > -this->cwHalf - 0.5 && c.Rear() < this->cwHalf + 0.5 &&
            c.v > 0.3)
          return true;                                  // passing right now
        if (front < -this->cwHalf && c.v > 1.0 &&
            (-this->cwHalf - front) / c.v < 3.5)
          return true;                                  // arriving soon
      }
    }
    return false;
  }

  void StartCrossing(Pedestrian &_p)
  {
    this->ReleaseSlot(_p);
    _p.stage = PedStage::kCrossing;
    const int crossDir = -_p.startSide;
    _p.target = Vector2d(_p.crossX, crossDir * (this->roadHalf + 0.6));
  }

  // ======================================================== referee
  int ZoneOf(double _y) const
  {
    if (_y < -this->roadHalf) return -1;
    if (_y > this->roadHalf) return 1;
    return 0;
  }

  void UpdateRobot(EntityComponentManager &_ecm)
  {
    if (this->robot != kNullEntity && !_ecm.HasEntity(this->robot))
      this->robot = kNullEntity;
    if (this->robot == kNullEntity)
    {
      this->robot = _ecm.EntityByComponents(
        components::Model(), components::Name(this->prm.robotName));
      if (this->robot == kNullEntity) return;
      this->robotPose = gz::sim::worldPose(this->robot, _ecm);
      this->robotZone = this->ZoneOf(this->robotPose.Pos().Y());
      this->ClearPedestriansAround(
        Vector2d(this->robotPose.Pos().X(), this->robotPose.Pos().Y()), 1.2);
      gzmsg << "[CrosswalkScenario] tracking robot [" << this->prm.robotName
            << "]" << std::endl;
      this->Event("{\"type\":\"robot_detected\"}");
      return;
    }
    this->robotPose = gz::sim::worldPose(this->robot, _ecm);
  }

  Obb RobotObb() const
  {
    Obb b;
    b.c = Vector2d(this->robotPose.Pos().X(), this->robotPose.Pos().Y());
    b.yaw = this->robotPose.Rot().Yaw();
    b.hl = this->prm.robotLen / 2;
    b.hw = this->prm.robotWid / 2;
    return b;
  }

  void Violation(const std::string &_what, const std::string &_extra = "")
  {
    for (const auto &v : this->attemptViolations)
      if (v == _what) return;
    this->attemptViolations.push_back(_what);
    this->Event("{\"type\":\"violation\",\"violation\":" + Str(_what) +
                (_extra.empty() ? "" : "," + _extra) + "}");
  }

  void UpdateReferee()
  {
    if (this->robot == kNullEntity)
      return;
    const auto &p = this->prm;
    const Obb rb = this->RobotObb();
    const int zone = this->ZoneOf(rb.c.Y());
    const int startSide = p.robotSpawn.Pos().Y() < 0 ? -1 : 1;
    const std::string robotXY =
      "\"robot\":{\"x\":" + Num(rb.c.X()) + ",\"y\":" + Num(rb.c.Y()) + "}";

    // ---- zone transitions
    if (zone != this->robotZone)
    {
      if (zone == 0 && !this->attemptActive)
      {
        this->attemptActive = true;
        ++this->attempts;
        this->attemptStart = this->now;
        this->attemptViolations.clear();
        this->attemptCollisions = 0;
        this->attemptFrom = this->robotZone;
        const std::string light = this->PedLight();
        this->Event("{\"type\":\"entered_road\",\"attempt\":" +
          std::to_string(this->attempts) + ",\"pedestrian_light\":" +
          Str(light) + ",\"light_remaining\":" + Num(this->PedLightRemaining()) +
          "," + robotXY + "}");
        if (light == "RED")
          this->Violation("entered_on_red");
        else if (light == "GREEN_BLINKING")
          this->Event("{\"type\":\"warning\",\"warning\":"
                      "\"entered_on_flashing_green\"}");
      }
      else if (zone != 0 && this->attemptActive)
      {
        this->attemptActive = false;
        const double dur = this->now - this->attemptStart;
        std::string list;
        for (const auto &v : this->attemptViolations)
          list += (list.empty() ? "" : ",") + Str(v);
        if (zone != this->attemptFrom)
        {
          const bool ok =
            this->attemptCollisions == 0 && this->attemptViolations.empty();
          this->lastResult = ok ? "SUCCESS" : "FAILED";
          if (ok) ++this->successes;
          this->Event("{\"type\":\"crossed\",\"result\":" +
            Str(this->lastResult) + ",\"attempt\":" +
            std::to_string(this->attempts) + ",\"time_on_road\":" + Num(dur) +
            ",\"collisions\":" + std::to_string(this->attemptCollisions) +
            ",\"violations\":[" + list + "],\"to_side\":" +
            Str(zone > 0 ? "north" : "south") + "}");
        }
        else
        {
          this->lastResult = "ABORTED";
          this->Event("{\"type\":\"aborted\",\"attempt\":" +
            std::to_string(this->attempts) + ",\"time_on_road\":" + Num(dur) +
            ",\"collisions\":" + std::to_string(this->attemptCollisions) +
            ",\"violations\":[" + list + "]}");
        }
      }
      this->robotZone = zone;
    }

    if (this->attemptActive && std::abs(rb.c.X()) > this->cwHalf + 0.25)
      this->Violation("outside_crosswalk", robotXY);

    // ---- goal
    const double goalDist =
      rb.c.Distance(Vector2d(p.goal.X(), p.goal.Y()));
    if (!this->goalReported && goalDist < 0.5 && zone == -startSide)
    {
      this->goalReported = true;
      this->Event("{\"type\":\"goal_reached\",\"time\":" + Num(this->now) +
                  "}");
    }
    else if (this->goalReported && goalDist > 1.5)
      this->goalReported = false;

    // ---- collisions with cars / near misses
    for (auto &c : this->cars)
    {
      if (!c.active) { c.contact = false; continue; }
      const Obb cb = this->CarObb(c);
      if (std::abs(cb.c.X() - rb.c.X()) > 12.0)
      {
        c.contact = false;
        continue;
      }
      const double d = ObbDistance(rb, cb);
      const bool hit = d <= 0.0;
      if (hit && !c.contact)
      {
        ++this->collisionsTotal;
        ++this->attemptCollisions;
        this->Event("{\"type\":\"collision\",\"with\":\"car\",\"id\":" +
          Str(c.name) + ",\"car_speed\":" + Num(c.v) + ",\"red_runner\":" +
          (c.runner ? "true" : "false") + ",\"pedestrian_light\":" +
          Str(this->PedLight()) + "," + robotXY + "}");
        this->PauseIfNeeded();
      }
      c.contact = hit;
      if (!hit && zone == 0 && d < p.nearMiss && !c.nearMiss && c.v > 0.5)
      {
        c.nearMiss = true;
        this->Event("{\"type\":\"near_miss\",\"id\":" + Str(c.name) +
          ",\"distance\":" + Num(d) + ",\"car_speed\":" + Num(c.v) +
          ",\"red_runner\":" + (c.runner ? "true" : "false") + "}");
      }
    }
    // ---- collisions with pedestrians
    for (auto &pd : this->peds)
    {
      if (!pd.active) { pd.contact = false; continue; }
      const bool hit = PointObbDistance(pd.pos, rb) < 0.2;
      if (hit && !pd.contact)
      {
        ++this->collisionsTotal;
        if (this->attemptActive) ++this->attemptCollisions;
        this->Event("{\"type\":\"collision\",\"with\":\"pedestrian\",\"id\":" +
          Str(pd.name) + "," + robotXY + "}");
        this->PauseIfNeeded();
      }
      pd.contact = hit;
    }
  }

  void PauseIfNeeded()
  {
    if (!this->prm.pauseOnCollision || this->worldName.empty())
      return;
    gz::msgs::WorldControl req;
    req.set_pause(true);
    std::function<void(const gz::msgs::Boolean &, const bool)> cb =
      [](const gz::msgs::Boolean &, const bool) {};
    this->node.Request("/world/" + this->worldName + "/control", req, cb);
  }

  // ===================================================== ECM output
  void SetModelPose(EntityComponentManager &_ecm, Entity _e, const Pose3d &_p)
  {
    auto pc = _ecm.Component<components::Pose>(_e);
    if (!pc) return;
    *pc = components::Pose(_p);
    _ecm.SetChanged(_e, components::Pose::typeId,
                    ComponentState::PeriodicChange);
  }

  void WriteEcm(EntityComponentManager &_ecm, bool _force)
  {
    for (auto &lamp : this->lamps)
    {
      const bool on = this->LampOn(lamp.group);
      if (!_force && lamp.known && lamp.on == on) continue;
      lamp.on = on;
      lamp.known = true;
      Pose3d pose = lamp.onPose;
      if (!on) pose.Pos().Z(kHiddenZ);
      this->SetModelPose(_ecm, lamp.entity, pose);
      _ecm.SetChanged(lamp.entity, components::Pose::typeId,
                      ComponentState::OneTimeChange);
    }

    for (auto &c : this->cars)
    {
      if (!c.dirty && !_force) continue;
      c.dirty = false;
      Pose3d pose(0, 0, kHiddenZ, 0, 0, 0);
      if (c.active)
      {
        const int dir = this->LaneDir(c.lane);
        pose = Pose3d(dir * c.s, this->LaneY(c.lane), 0.0, 0, 0,
                      dir > 0 ? 0.0 : kPi);
      }
      this->SetModelPose(_ecm, c.entity, pose);
    }

    for (auto &pd : this->peds)
    {
      if (!pd.dirty && !_force) continue;
      pd.dirty = false;
      Pose3d traj(0, 0, kHiddenZ, 0, 0, 0);
      if (pd.active)
        traj = Pose3d(pd.pos.X(), pd.pos.Y(), 0.0, 0, 0, pd.yaw);
      if (auto tc = _ecm.Component<components::TrajectoryPose>(pd.entity))
      {
        *tc = components::TrajectoryPose(traj);
        _ecm.SetChanged(pd.entity, components::TrajectoryPose::typeId,
                        ComponentState::PeriodicChange);
      }
      if (auto ac = _ecm.Component<components::AnimationTime>(pd.entity))
      {
        *ac = components::AnimationTime(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(pd.animTime)));
        _ecm.SetChanged(pd.entity, components::AnimationTime::typeId,
                        ComponentState::PeriodicChange);
      }
      if (pd.animNameDirty || _force)
      {
        pd.animNameDirty = false;
        if (auto nc = _ecm.Component<components::AnimationName>(pd.entity))
        {
          *nc = components::AnimationName(pd.walking ? "walk" : "stand");
          _ecm.SetChanged(pd.entity, components::AnimationName::typeId,
                          ComponentState::OneTimeChange);
        }
      }
    }
  }

  // ====================================================== publishing
  void Event(const std::string &_json)
  {
    // prepend sim time
    std::string msg = _json;
    if (msg.size() > 1 && msg.front() == '{')
      msg = "{\"t\":" + Num(this->now, 2) + "," + msg.substr(1);
    gz::msgs::StringMsg m;
    m.set_data(msg);
    this->pubEvents.Publish(m);
    gzmsg << "[CrosswalkScenario] " << msg << std::endl;
  }

  void Publish()
  {
    const std::string ped = this->PedLight();
    const std::string car = this->CarLight();
    gz::msgs::StringMsg m;
    if (ped != this->lastPedLight || this->now - this->lastLightPub > 1.0)
    {
      m.set_data(ped);
      this->pubPedLight.Publish(m);
      m.set_data(car);
      this->pubCarLight.Publish(m);
      this->lastPedLight = ped;
      this->lastCarLight = car;
      this->lastLightPub = this->now;
    }
    else if (car != this->lastCarLight)
    {
      m.set_data(car);
      this->pubCarLight.Publish(m);
      this->lastCarLight = car;
    }

    if (this->now - this->lastStatePub >= 0.1 - 1e-6)
    {
      this->lastStatePub = this->now;
      std::ostringstream ss;
      ss << "{\"t\":" << Num(this->now, 2)
         << ",\"pedestrian_light\":" << Str(ped)
         << ",\"car_light\":" << Str(car)
         << ",\"pedestrian_light_remaining\":" << Num(this->PedLightRemaining(), 2);
      if (this->robot != kNullEntity)
        ss << ",\"robot\":{\"x\":" << Num(this->robotPose.Pos().X())
           << ",\"y\":" << Num(this->robotPose.Pos().Y())
           << ",\"yaw\":" << Num(this->robotPose.Rot().Yaw()) << "}";
      ss << ",\"pedestrians\":[";
      bool first = true;
      for (const auto &pd : this->peds)
      {
        if (!pd.active) continue;
        ss << (first ? "" : ",") << "{\"id\":" << Str(pd.name)
           << ",\"x\":" << Num(pd.pos.X()) << ",\"y\":" << Num(pd.pos.Y())
           << ",\"yaw\":" << Num(pd.yaw) << ",\"vx\":" << Num(pd.vel.X())
           << ",\"vy\":" << Num(pd.vel.Y()) << ",\"state\":"
           << Str(pd.stage == PedStage::kWaiting ? "waiting" :
                  pd.stage == PedStage::kCrossing ? "crossing" : "walking")
           << ",\"jaywalker\":" << (pd.jaywalker ? "true" : "false") << "}";
        first = false;
      }
      ss << "],\"cars\":[";
      first = true;
      for (const auto &c : this->cars)
      {
        if (!c.active) continue;
        const Obb b = this->CarObb(c);
        ss << (first ? "" : ",") << "{\"id\":" << Str(c.name)
           << ",\"type\":" << Str(c.type)
           << ",\"x\":" << Num(b.c.X()) << ",\"y\":" << Num(b.c.Y())
           << ",\"yaw\":" << Num(b.yaw) << ",\"speed\":" << Num(c.v)
           << ",\"length\":" << Num(c.length) << ",\"width\":" << Num(c.width)
           << ",\"red_runner\":" << (c.runner ? "true" : "false") << "}";
        first = false;
      }
      ss << "]}";
      m.set_data(ss.str());
      this->pubState.Publish(m);
    }

    if (this->now - this->lastRefereePub >= 0.5 - 1e-6)
    {
      this->lastRefereePub = this->now;
      const char *zoneName[] = {"south_sidewalk", "road", "north_sidewalk"};
      std::ostringstream ss;
      ss << "{\"t\":" << Num(this->now, 2)
         << ",\"robot_found\":" << (this->robot != kNullEntity ? "true" : "false")
         << ",\"zone\":" << Str(zoneName[this->robotZone + 1])
         << ",\"attempt_active\":" << (this->attemptActive ? "true" : "false")
         << ",\"attempts\":" << this->attempts
         << ",\"successes\":" << this->successes
         << ",\"collisions\":" << this->collisionsTotal
         << ",\"last_result\":" << Str(this->lastResult);
      if (this->robot != kNullEntity)
        ss << ",\"goal_distance\":" << Num(Vector2d(
                this->robotPose.Pos().X(), this->robotPose.Pos().Y())
                .Distance(this->prm.goal), 2);
      ss << "}";
      m.set_data(ss.str());
      this->pubReferee.Publish(m);
    }
  }

  // ========================================================== data
  struct Params
  {
    int seed{0};
    double warmup{30};
    double laneWidth{3.5}, sidewalkWidth{4.0}, crosswalkWidth{4.0};
    double stopLineOffset{3.0}, roadLength{200.0};
    double tCarGreen{25}, tCarYellow{3}, tAllRed{2}, tPedGreen{12};
    double tPedBlink{4}, blinkPeriod{1}, tCarRedYellow{1.5};
    std::string startPhase{"random"};
    double pedCrossRate{8}, pedWalkRate{6}, pedSpeedMin{1.0}, pedSpeedMax{1.6};
    double pedRedProb{0.0}, pedSpawnDist{35}, animFactor{4.588};
    bool pedAvoidRobot{true};
    double carRate{8}, carSpeedMin{8}, carSpeedMax{13.9};
    bool carYieldRobot{false}, carYieldPed{true};
    double runnerProb{0.3}, runnerWinMin{0}, runnerWinMax{8};
    double runnerSpeedMin{10}, runnerSpeedMax{14};
    std::string robotName{"delivery_robot"};
    Pose3d robotSpawn;
    double robotLen{0.72}, robotWid{0.62};
    Vector2d goal{0, 6};
    double nearMiss{1.0};
    bool pauseOnCollision{false};
  } prm;

  Entity world{kNullEntity};
  std::string worldName;
  unsigned seedUsed{0};
  std::mt19937 rng;

  bool resolved{false};
  int resolveAttempts{0};
  bool initialized{false};
  double actorZ{1.0};

  double now{0}, lastSimTime{0}, accum{0};
  double roadHalf{3.5}, cwHalf{2.0}, stopLineS{-5.0}, spawnS{-95.0};

  Phase phase{Phase::kCarGreen};
  double phaseT{0};

  std::vector<Pedestrian> peds;
  std::vector<Car> cars;
  std::vector<Lamp> lamps;
  // waiting slots [side][row][col]
  std::array<std::array<std::array<bool, kCols>, kRows>, 2> slots{};

  double nextCrosser{0}, nextWalker{0};
  std::array<double, 2> nextCar{0, 0};
  std::array<bool, 2> laneBlocked{false, false};
  bool runnerDecided{false};
  PendingRunner runner;

  // robot / referee
  Entity robot{kNullEntity};
  Pose3d robotPose;
  int robotZone{-1};
  bool attemptActive{false};
  int attemptFrom{-1};
  double attemptStart{0};
  int attempts{0}, successes{0}, collisionsTotal{0}, attemptCollisions{0};
  std::vector<std::string> attemptViolations;
  std::string lastResult{"NONE"};
  bool goalReported{false};
  std::atomic<bool> resetRequested{false};

  // transport
  gz::transport::Node node;
  gz::transport::Node::Publisher pubPedLight, pubCarLight, pubState,
    pubEvents, pubReferee;
  std::string lastPedLight, lastCarLight;
  double lastLightPub{-1e9}, lastStatePub{-1e9}, lastRefereePub{-1e9};
};

}  // namespace crosswalk_sim

GZ_ADD_PLUGIN(crosswalk_sim::CrosswalkScenario,
              gz::sim::System,
              crosswalk_sim::CrosswalkScenario::ISystemConfigure,
              crosswalk_sim::CrosswalkScenario::ISystemPreUpdate,
              crosswalk_sim::CrosswalkScenario::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(crosswalk_sim::CrosswalkScenario,
                    "crosswalk_sim::CrosswalkScenario")
