#include "neurovector/avoidance.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace nav = neurovector::avoidance;

namespace {
struct Result {
  std::string name, action, expected;
  std::size_t detour{};
};

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

nav::Obstacle read_obstacle() {
  nav::Obstacle obstacle;
  require(static_cast<bool>(std::cin >> obstacle.id >> obstacle.priority >>
                            obstacle.position.x >> obstacle.position.y >>
                            obstacle.position.z >> obstacle.radius_m),
          "expected three fixture obstacle rows: id priority x y z radius");
  obstacle.confidence = 0.95;
  obstacle.observed_at_s = 10;
  obstacle.position_known = true;
  obstacle.velocity_known = true;
  return obstacle;
}

nav::Input base_input() {
  nav::Input input;
  input.now_s = 10;
  input.sensors_ready = true;
  input.observations_at_s = 10;
  input.position = {0, 0, 8};
  input.velocity = {2, 0, 0};
  input.remaining_route = {{30, 0, 8}};
  input.coverage = {{{{-50, -50, -10}, {100, 100, 50}}, 10}};
  return input;
}

nav::Decision check(std::vector<Result> &results, const std::string &name,
                    const nav::Input &input, nav::Action expected,
                    const nav::Config &config) {
  const auto decision = nav::decide(input, config);
  require(decision.action == expected,
          name + ": expected " + nav::action_name(expected) + ", got " +
              nav::action_name(decision.action) + " (" + decision.reason + ")");
  results.push_back({name, nav::action_name(decision.action),
                     nav::action_name(expected), decision.detour.size()});
  return decision;
}

double distance(nav::Vec3 a, nav::Vec3 b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

void check_projection(std::vector<Result> &results, double altitude) {
  nav::Camera camera;
  camera.fx = camera.fy = 400;
  camera.cx = 320;
  camera.cy = 240;
  camera.width = 640;
  camera.height = 480;
  camera.camera_to_enu = {1, 0, 0, 0, 0, 1, 0, -1, 0};
  camera.origin_enu = {0, 0, altitude};
  camera.pose_valid = true;
  const nav::DetectionBox box{400, 230, 420, 250};
  const double radial = 10;
  auto projected = nav::project_detection(box, radial, camera);
  require(projected.has_value(), "proper camera pose must project detection");
  require(
      std::abs(distance(*projected, camera.origin_enu) - radial) < 1e-9,
      "radial projection must normalize ray, rather than treating range as Z");
  require(projected->x > 0 && projected->y > 0 &&
              std::abs(projected->z - altitude) < 1e-9,
          "camera rotation must map right/forward to east/north");
  results.push_back({"radial_projection", "projected", "projected", 0});

  camera.mirrored = true;
  const nav::DetectionBox mirrored{camera.width - 1 - box.x2, box.y1,
                                   camera.width - 1 - box.x1, box.y2};
  auto reflected = nav::project_detection(mirrored, radial, camera);
  require(reflected && distance(*projected, *reflected) < 1e-9,
          "mirrored image must preserve physical ENU projection");
  results.push_back({"mirrored_projection", "projected", "projected", 0});

  require(!nav::project_detection(box, std::nullopt, camera),
          "missing depth must remain unlocalized");
  camera.pose_valid = false;
  require(!nav::project_detection(box, radial, camera),
          "unknown pose must not fabricate an ENU origin obstacle");
  camera.pose_valid = true;
  camera.camera_to_enu[0] = -1;
  require(!nav::project_detection(box, radial, camera),
          "improper reflection matrix must be rejected");
  results.push_back({"invalid_projection", "unlocalized", "unlocalized", 0});
}
} // namespace

int main() {
  try {
    const auto student = read_obstacle();
    const auto van = read_obstacle();
    const auto leaves = read_obstacle();
    require(student.priority == 1 && van.priority == 2 && leaves.priority == 3,
            "fixture obstacle rows must be ordered person, vehicle, leaves");
    nav::Vec3 window;
    double bearing{}, standoff{}, drone_radius{};
    require(
        static_cast<bool>(std::cin >> window.x >> window.y >> window.z >>
                          bearing >> standoff >> drone_radius),
        "expected window x y z outward_bearing standoff, then drone radius");
    std::string trailing;
    require(!(std::cin >> trailing), "unexpected trailing fixture input");
    nav::Config config;
    config.drone_radius_m = drone_radius;
    std::vector<Result> results;

    auto input = base_input();
    check(results, "fresh_empty_corridor", input, nav::Action::proceed, config);
    input.obstacles = {student};
    check(results, "on_path_student", input, nav::Action::hold, config);
    input.obstacles = {van};
    check(results, "vehicle_far_ahead", input, nav::Action::slow, config);
    input.obstacles = {leaves};
    check(results, "leaves_off_route", input, nav::Action::proceed, config);

    auto distant_student = student;
    distant_student.position.x = student.position.x * 4;
    auto imminent_leaf = leaves;
    imminent_leaf.position = {student.position.x / 3, input.position.y,
                              input.position.z};
    input.obstacles = {distant_student, imminent_leaf};
    auto decision = check(results, "imminent_leaf_overrides_priority", input,
                          nav::Action::hold, config);
    require(decision.obstacle_index == 1 && !decision.threat_order.empty() &&
                decision.threat_order.front() == 1,
            "immediate tier-3 collision must precede the distant person");

    input = base_input();
    input.observations_at_s = input.now_s - config.max_age_s - 1;
    check(results, "stale_sensor_heartbeat", input, nav::Action::hold, config);
    input = base_input();
    input.sensors_ready = false;
    check(results, "missing_sensor_pipeline", input, nav::Action::hold, config);
    input = base_input();
    auto unknown = student;
    unknown.position_known = false;
    unknown.distance_m.reset();
    input.obstacles = {unknown};
    check(results, "unknown_depth", input, nav::Action::hold, config);
    unknown.distance_m = 1000;
    unknown.observed_at_s = 0;
    input.obstacles = {unknown};
    check(results, "stale_unknown_far_range", input, nav::Action::hold, config);

    input = base_input();
    const double radians = bearing * std::acos(-1.0) / 180;
    const nav::Vec3 outward{std::sin(radians), std::cos(radians), 0};
    input.position = {window.x + outward.x * standoff,
                      window.y + outward.y * standoff, window.z};
    input.velocity = {};
    input.remaining_route = {input.position};
    input.facade = nav::Facade{window, outward, 2};
    input.window_handoff = true;
    decision = check(results, "window_exterior_handoff", input,
                     nav::Action::hold, config);
    require(decision.desired_speed_mps == 0 && decision.detour.empty(),
            "window handoff must remain stationary without an interior detour");

    input = base_input();
    input.velocity.x = config.slow_speed_mps * 3;
    input.obstacles = {distant_student};
    auto detour_config = config;
    detour_config.horizon_s = 15;
    decision = check(results, "checked_lateral_detour", input,
                     nav::Action::request_replan, detour_config);
    require(
        !decision.detour.empty(),
        "fresh free-space coverage should allow a checked lateral candidate");
    for (auto point : decision.detour)
      require(std::abs(point.z - input.position.z) < 1e-9,
              "avoidance must not introduce an unvalidated altitude change");

    input.coverage.clear();
    decision = check(results, "missing_detour_coverage", input,
                     nav::Action::request_replan, detour_config);
    require(decision.detour.empty(),
            "unobserved detour must not be labeled safe");
    input = base_input();
    input.velocity.x = config.slow_speed_mps * 3;
    input.obstacles = {distant_student};
    const double offset = detour_config.detour_offset_m;
    input.no_fly_boxes = {{{offset, offset / 2, input.position.z - 1},
                           {distant_student.position.x + 2 * offset,
                            1.5 * offset, input.position.z + 1}},
                          {{offset, -1.5 * offset, input.position.z - 1},
                           {distant_student.position.x + 2 * offset,
                            -offset / 2, input.position.z + 1}}};
    decision = check(results, "both_detours_restricted", input,
                     nav::Action::request_replan, detour_config);
    require(decision.detour.empty(),
            "both restricted lateral corridors must reject detours");

    check_projection(results, base_input().position.z);
    std::cout << "{\"passed\":" << results.size() << ",\"cases\":[";
    for (std::size_t i = 0; i < results.size(); ++i) {
      if (i)
        std::cout << ',';
      const auto &result = results[i];
      std::cout << "{\"name\":" << std::quoted(result.name)
                << ",\"action\":" << std::quoted(result.action)
                << ",\"expected\":" << std::quoted(result.expected)
                << ",\"detour\":" << result.detour << '}';
    }
    std::cout << "]}\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "school scenario failed: " << error.what() << '\n';
    return 1;
  }
}
