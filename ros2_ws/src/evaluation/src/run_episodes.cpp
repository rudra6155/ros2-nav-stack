// run_episodes: the evaluation harness.
//
//   ros2 run evaluation run_episodes --world simple --episodes 20
//
// For one world it
//   1. starts the whole stack as a child process (`ros2 launch bringup sim.launch.py ... headless:=true`),
//   2. for each episode: samples a start/goal from the episode seed, places the robot there
//      (Gazebo /set_entity_state), resets the costmap, publishes the goal and measures
//        success (goal within 0.3 m), time_to_goal_s (sim time), path_length_m (driven, from the
//        ground-truth odometry), collision_count (Gazebo contact sensor on the chassis,
//        debounced), min_obstacle_distance_m (ground-truth distance centre -> nearest surface),
//        timed_out, seed,
//   3. supervises the child: a node that dies mid-episode marks that episode timed_out=true (the
//      node + its stderr go to crash_log.txt), the stack is restarted and the run continues. A
//      crashed episode is never dropped from the denominator,
//   4. writes episodes.csv (EXACT schema from SPEC.md), episode_diagnostics.csv and rtf_log.csv,
//      replacing only the rows of this world.
//
// Everything is simulation time; wall time is used only for watchdogs and for the RTF.
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "evaluation/episode_sampler.hpp"
#include "evaluation/world_geometry.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"

extern char ** environ;

namespace fs = std::filesystem;

using Clock = std::chrono::steady_clock;

namespace
{

// ------------------------------------------------------------------ options
struct Args
{
  std::string world{"simple"};
  int episodes{20};
  int only_episode{0};             // >0: run just this episode number (reproduce one seed)
  uint64_t seed_base{1000};
  std::string results_dir;
  std::string worlds_dir;
  std::string tag{"eval"};         // used in log/bag file names
  double hard_timeout_s{120.0};    // sim seconds
  double lookahead{0.6};
  double inflation{0.30};
  bool gui{false};                 // headless:=false (needs a display; used for the GIF)
  bool record_bag{true};           // rosbag of the whole run (results/bags/, manifest committed)
  bool no_launch{false};           // use an already running stack
  double goal_tolerance{0.3};
  double warmup_s{1.5};            // sim seconds of fresh mapping before the goal is published
  int ready_timeout_s{150};
  std::vector<std::string> launch_args;  // extra launch arguments, given as key=value (fault injection)
};

void usage()
{
  std::cerr <<
    "usage: run_episodes --world simple|cluttered [--episodes N] [--episode K] [--seed-base S]\n"
    "                    [--results-dir DIR] [--worlds-dir DIR] [--tag NAME] [--timeout SIM_S]\n"
    "                    [--lookahead M] [--inflation M] [--gui] [--no-bag] [--no-launch] [--launch-arg key=value]\n";
}

bool parse_args(int argc, char ** argv, Args & a)
{
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](std::string & out) {
        if (i + 1 >= argc) {return false;}
        out = argv[++i];
        return true;
      };
    std::string v;
    if (k == "--world") {if (!next(a.world)) {return false;}}
    else if (k == "--episodes") {if (!next(v)) {return false;} a.episodes = std::stoi(v);}
    else if (k == "--episode") {if (!next(v)) {return false;} a.only_episode = std::stoi(v);}
    else if (k == "--seed-base") {if (!next(v)) {return false;} a.seed_base = std::stoull(v);}
    else if (k == "--results-dir") {if (!next(a.results_dir)) {return false;}}
    else if (k == "--worlds-dir") {if (!next(a.worlds_dir)) {return false;}}
    else if (k == "--tag") {if (!next(a.tag)) {return false;}}
    else if (k == "--timeout") {if (!next(v)) {return false;} a.hard_timeout_s = std::stod(v);}
    else if (k == "--lookahead") {if (!next(v)) {return false;} a.lookahead = std::stod(v);}
    else if (k == "--inflation") {if (!next(v)) {return false;} a.inflation = std::stod(v);}
    else if (k == "--launch-arg") {if (!next(v)) {return false;} a.launch_args.push_back(v);}
    else if (k == "--gui") {a.gui = true;}
    else if (k == "--record-bag") {a.record_bag = true;}
    else if (k == "--no-bag") {a.record_bag = false;}
    else if (k == "--no-launch") {a.no_launch = true;}
    else if (k == "--help" || k == "-h") {return false;}
    else {std::cerr << "unknown argument: " << k << "\n"; return false;}
  }
  return a.episodes > 0 && (a.world == "simple" || a.world == "cluttered" || !a.world.empty());
}

std::string wall_stamp()
{
  const std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
  return buf;
}

void mkdirs(const std::string & path)
{
  std::string cur;
  std::istringstream ss(path);
  std::string part;
  if (!path.empty() && path[0] == '/') {cur = "/";}
  while (std::getline(ss, part, '/')) {
    if (part.empty()) {continue;}
    cur += part + "/";
    ::mkdir(cur.c_str(), 0755);
  }
}

void append_line(const std::string & path, const std::string & line)
{
  std::ofstream f(path, std::ios::app);
  f << line << "\n";
}

// ------------------------------------------------------------------ CSV helpers
std::string num(double v, int prec = 4)
{
  if (std::isnan(v)) {return "nan";}
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(prec) << v;
  return ss.str();
}

/// Replace all rows of `world` in `path` (first column) by `new_rows`; keep other worlds' rows.
void merge_rows(
  const std::string & path, const std::string & header, const std::string & world,
  const std::vector<std::string> & new_rows)
{
  std::vector<std::string> keep;
  {
    std::ifstream in(path);
    std::string line;
    bool first = true;
    while (std::getline(in, line)) {
      if (first) {first = false; continue;}
      if (line.empty()) {continue;}
      if (line.substr(0, line.find(',')) != world) {keep.push_back(line);}
    }
  }
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp);
    out << header << "\n";
    for (const auto & l : keep) {out << l << "\n";}
    for (const auto & l : new_rows) {out << l << "\n";}
  }
  std::rename(tmp.c_str(), path.c_str());
}

const char * EPISODES_HEADER =
  "world,episode,seed,success,time_to_goal_s,path_length_m,collision_count,"
  "min_obstacle_distance_m,timed_out";
const char * DIAG_HEADER =
  "world,episode,seed,start_x,start_y,start_yaw,goal_x,goal_y,straight_dist_m,duration_s,"
  "final_goal_dist_m,max_stationary_s,empty_plan_frac,crashed,stop_reason,rtf,timeout_cap_s";
const char * RTF_HEADER = "world,episode,rtf";

// ------------------------------------------------------------------ child process (the stack)
struct DeathInfo
{
  std::string process;   // e.g. "astar_planner-6"
  std::string line;      // the launch "process has died" line
  std::string stderr_tail;
};

class StackProcess
{
public:
  bool start(const std::vector<std::string> & argv, const std::string & log_path)
  {
    stop();
    log_path_ = log_path;
    offset_ = 0;
    int fd = ::open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {return false;}
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, fd, 1);
    posix_spawn_file_actions_adddup2(&fa, fd, 2);
    posix_spawn_file_actions_addclose(&fa, fd);
    posix_spawnattr_t attr;
    posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);  // own session/process group: kill it all at once
    std::vector<char *> cargv;
    for (const auto & s : argv) {cargv.push_back(const_cast<char *>(s.c_str()));}
    cargv.push_back(nullptr);
    const int rc = posix_spawnp(&pid_, cargv[0], &fa, &attr, cargv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    posix_spawnattr_destroy(&attr);
    ::close(fd);
    if (rc != 0) {pid_ = -1; return false;}
    return true;
  }

  bool alive()
  {
    if (pid_ <= 0) {return false;}
    int status = 0;
    const pid_t r = ::waitpid(pid_, &status, WNOHANG);
    if (r == pid_) {pid_ = -1; return false;}
    return true;
  }

  void stop()
  {
    if (pid_ > 0) {
      ::kill(-pid_, SIGINT);
      for (int i = 0; i < 200; ++i) {  // up to 20 s for a clean shutdown
        int status = 0;
        if (::waitpid(pid_, &status, WNOHANG) == pid_) {pid_ = -1; break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (pid_ > 0) {
        ::kill(-pid_, SIGKILL);
        int status = 0;
        ::waitpid(pid_, &status, 0);
        pid_ = -1;
      }
    }
    // gzserver/gzclient can outlive a killed launch; they would hold Gazebo's master port
    std::system("pkill -9 -x gzserver >/dev/null 2>&1; pkill -9 -x gzclient >/dev/null 2>&1");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  /// New "process has died" events since the last call.
  std::vector<DeathInfo> poll_deaths()
  {
    std::vector<DeathInfo> out;
    std::ifstream in(log_path_);
    if (!in) {return out;}
    in.seekg(0, std::ios::end);
    const std::streamoff size = in.tellg();
    if (size <= offset_) {return out;}
    in.seekg(0);
    std::vector<std::string> lines;
    std::string line;
    std::streamoff pos = 0;
    while (std::getline(in, line)) {
      pos += static_cast<std::streamoff>(line.size()) + 1;
      lines.push_back(line);
    }
    // lines before offset_ were already scanned; keep them only as context for stderr_tail
    std::streamoff scanned = 0;
    for (size_t i = 0; i < lines.size(); ++i) {
      scanned += static_cast<std::streamoff>(lines[i].size()) + 1;
      if (scanned <= offset_) {continue;}
      const auto d = lines[i].find("process has died");
      if (d == std::string::npos) {continue;}
      DeathInfo di;
      di.line = lines[i];
      const auto b = lines[i].find("] [");
      const auto e = lines[i].find("]:", b == std::string::npos ? 0 : b + 3);
      if (b != std::string::npos && e != std::string::npos) {
        di.process = lines[i].substr(b + 3, e - (b + 3));
      }
      const std::string prefix = "[" + di.process + "]";
      std::vector<std::string> tail;
      for (size_t j = 0; j < i; ++j) {
        if (!di.process.empty() && lines[j].rfind(prefix, 0) == 0) {tail.push_back(lines[j]);}
      }
      const size_t from = tail.size() > 30 ? tail.size() - 30 : 0;
      for (size_t j = from; j < tail.size(); ++j) {di.stderr_tail += tail[j] + "\n";}
      out.push_back(di);
    }
    offset_ = pos;
    return out;
  }

  bool has_pid() const {return pid_ > 0;}

private:
  pid_t pid_{-1};
  std::string log_path_;
  std::streamoff offset_{0};
};

// ------------------------------------------------------------------ episode bookkeeping
struct EpisodeRecord
{
  int episode{0};
  uint64_t seed{0};
  evaluation::EpisodePoses poses;
  bool success{false};
  double time_to_goal_s{std::nan("")};
  double path_length_m{0.0};
  int collision_count{0};
  double min_obstacle_distance_m{std::nan("")};
  bool timed_out{false};
  bool crashed{false};
  std::string stop_reason;
  double duration_s{0.0};
  double final_goal_dist_m{std::nan("")};
  double max_stationary_s{0.0};
  double empty_plan_frac{0.0};
  double rtf{std::nan("")};
  double timeout_cap_s{0.0};
};

class Runner : public rclcpp::Node
{
public:
  explicit Runner(const Args & args)
  : Node("run_episodes", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
    args_(args)
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10, [this](nav_msgs::msg::Path::ConstSharedPtr m) {on_plan(*m);});
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "costmap", rclcpp::QoS(1), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {++costmap_count_;});
    bumper_sub_ = create_subscription<gazebo_msgs::msg::ContactsState>(
      "bumper_states", rclcpp::SensorDataQoS(),
      [this](gazebo_msgs::msg::ContactsState::ConstSharedPtr m) {on_bumper(*m);});
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    cancel_pub_ = create_publisher<std_msgs::msg::Empty>("goal_cancel", 10);
    set_state_ = create_client<gazebo_msgs::srv::SetEntityState>("set_entity_state");
    reset_map_ = create_client<std_srvs::srv::Empty>("reset_costmap");
  }

  // ---- counters used by the readiness check
  int odom_count() const {return odom_count_;}
  int costmap_count() const {return costmap_count_;}
  void clear_counters() {odom_count_ = 0; costmap_count_ = 0;}
  bool services_ready() const {return set_state_->service_is_ready() && reset_map_->service_is_ready();}

  bool wait_ready(int timeout_s)
  {
    const auto t0 = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - t0).count() < timeout_s) {
      const bool ok = now().nanoseconds() > 0 && odom_count_ >= 5 && costmap_count_ >= 1 &&
        services_ready() && count_publishers("plan") >= 1 && count_publishers("cmd_vel") >= 1;
      if (ok) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    return false;
  }

  // ---- snapshot of the sim clock (works across stack restarts)
  double sim_now() {return now().seconds();}

  void publish_cancel() {cancel_pub_->publish(std_msgs::msg::Empty());}

  bool teleport(double x, double y, double yaw)
  {
    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state.name = "diffbot";
    req->state.pose.position.x = x;
    req->state.pose.position.y = y;
    req->state.pose.position.z = 0.06;
    req->state.pose.orientation.z = std::sin(yaw / 2.0);
    req->state.pose.orientation.w = std::cos(yaw / 2.0);
    req->state.reference_frame = "world";
    auto prom = std::make_shared<std::promise<bool>>();
    auto fut = prom->get_future();
    set_state_->async_send_request(
      req, [prom](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture f) {
        prom->set_value(f.get()->success);
      });
    if (fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {return false;}
    return fut.get();
  }

  bool reset_costmap()
  {
    auto prom = std::make_shared<std::promise<void>>();
    auto fut = prom->get_future();
    reset_map_->async_send_request(
      std::make_shared<std_srvs::srv::Empty::Request>(),
      [prom](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {prom->set_value();});
    return fut.wait_for(std::chrono::seconds(10)) == std::future_status::ready;
  }

  bool robot_near(double x, double y, double tol)
  {
    std::lock_guard<std::mutex> l(mutex_);
    return have_odom_ && std::hypot(odom_x_ - x, odom_y_ - y) < tol && odom_speed_ < 0.05;
  }

  // ---- episode measurement window
  void begin(double gx, double gy)
  {
    std::lock_guard<std::mutex> l(mutex_);
    m_ = Metrics();
    m_.goal_x = gx;
    m_.goal_y = gy;
    m_.t0 = have_odom_ ? odom_t_ : 0.0;
    m_.last_t = m_.t0;
    m_.last_x = odom_x_;
    m_.last_y = odom_y_;
    m_.have_last = true;
    m_.active = true;
  }

  struct Metrics
  {
    bool active{false};
    double goal_x{0}, goal_y{0};
    double t0{0};
    double last_x{0}, last_y{0};
    bool have_last{false};
    double path_length{0};
    double min_obstacle_distance{std::numeric_limits<double>::infinity()};
    int collisions{0};
    double last_contact_t{-1e9};
    double stationary_streak{0};
    double max_stationary{0};
    double last_t{0};
    bool reached{false};
    double reach_time{0};
    double final_dist{std::nan("")};
    size_t plans{0}, plans_empty{0};
  };

  Metrics snapshot()
  {
    std::lock_guard<std::mutex> l(mutex_);
    return m_;
  }
  void end_measurement()
  {
    std::lock_guard<std::mutex> l(mutex_);
    m_.active = false;
  }

  void publish_goal(double x, double y)
  {
    geometry_msgs::msg::PoseStamped g;
    g.header.frame_id = "map";
    g.header.stamp = now();
    g.pose.position.x = x;
    g.pose.position.y = y;
    g.pose.orientation.w = 1.0;
    goal_pub_->publish(g);
  }

  void set_geometry(const evaluation::WorldGeometry * geo) {geo_ = geo;}

private:
  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    std::lock_guard<std::mutex> l(mutex_);
    ++odom_count_;
    const double t = rclcpp::Time(m.header.stamp).seconds();
    odom_x_ = m.pose.pose.position.x;
    odom_y_ = m.pose.pose.position.y;
    odom_speed_ = std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y);
    odom_t_ = t;
    have_odom_ = true;
    if (!m_.active) {return;}

    const double dt = std::max(0.0, t - m_.last_t);
    if (m_.have_last) {
      const double d = std::hypot(odom_x_ - m_.last_x, odom_y_ - m_.last_y);
      if (d < 0.5) {m_.path_length += d;}  // ignore any teleport-sized jump
    }
    m_.last_x = odom_x_;
    m_.last_y = odom_y_;
    if (geo_) {
      m_.min_obstacle_distance = std::min(m_.min_obstacle_distance, geo_->distance(odom_x_, odom_y_));
    }
    // stationary streaks (ignore the first 2 s while the first plan is computed)
    if (t - m_.t0 > 2.0 && m_.last_t > 0.0) {
      if (odom_speed_ < 0.03) {
        m_.stationary_streak += dt;
        m_.max_stationary = std::max(m_.max_stationary, m_.stationary_streak);
      } else {
        m_.stationary_streak = 0.0;
      }
    }
    m_.last_t = t;
    m_.final_dist = std::hypot(m_.goal_x - odom_x_, m_.goal_y - odom_y_);
    if (!m_.reached && m_.final_dist <= args_.goal_tolerance) {
      m_.reached = true;
      m_.reach_time = t - m_.t0;
    }
  }

  void on_plan(const nav_msgs::msg::Path & p)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (!m_.active) {return;}
    ++m_.plans;
    if (p.poses.empty()) {++m_.plans_empty;}
  }

  void on_bumper(const gazebo_msgs::msg::ContactsState & c)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (!m_.active || c.states.empty()) {return;}
    const double t = rclcpp::Time(c.header.stamp).seconds();
    if (t - m_.last_contact_t > 0.5) {++m_.collisions;}  // new contact event (debounced)
    m_.last_contact_t = t;
  }

  Args args_;
  const evaluation::WorldGeometry * geo_{nullptr};
  std::mutex mutex_;
  Metrics m_;
  bool have_odom_{false};
  double odom_x_{0}, odom_y_{0}, odom_speed_{0}, odom_t_{0};
  std::atomic<int> odom_count_{0};
  std::atomic<int> costmap_count_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bumper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_map_;
};

std::string fmt_pose(double x, double y)
{
  std::ostringstream ss;
  ss << "(" << std::fixed << std::setprecision(2) << x << ", " << y << ")";
  return ss.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  rclcpp::init(argc, argv);
  const auto remaining = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> cargv;
  for (const auto & s : remaining) {cargv.push_back(const_cast<char *>(s.c_str()));}
  try {
    if (!parse_args(static_cast<int>(cargv.size()), cargv.data(), args)) {usage(); return 2;}
  } catch (const std::exception & e) {
    std::cerr << "bad argument: " << e.what() << "\n";
    usage();
    return 2;
  }

  if (args.results_dir.empty()) {
    const char * env = std::getenv("NAV_RESULTS_DIR");
    args.results_dir = env ? env : (fs::exists("/workspace/results") ? "/workspace/results" : "results");
  }
  if (args.worlds_dir.empty()) {
    args.worlds_dir = ament_index_cpp::get_package_share_directory("bringup") + "/worlds";
  }
  mkdirs(args.results_dir + "/logs");
  const std::string crash_log = args.results_dir + "/crash_log.txt";
  const std::string limits_report = args.results_dir + "/limits_report.txt";

  evaluation::WorldGeometry geo;
  try {
    geo = evaluation::WorldGeometry::load(args.worlds_dir + "/" + args.world + ".obstacles");
  } catch (const std::exception & e) {
    std::cerr << "FATAL: " << e.what() << "\n";
    return 2;
  }
  evaluation::SamplerConfig scfg;
  // Episodes are always defined with the nominal 0.30 m inflation, whatever --inflation tests, so
  // that a sensitivity sweep runs the SAME start/goal pairs for every parameter value.
  const evaluation::EpisodeSampler sampler(geo, scfg);

  // episode numbers to run (1-based; seed = seed_base + episode)
  std::vector<int> numbers;
  if (args.only_episode > 0) {
    numbers.push_back(args.only_episode);
  } else {
    for (int e = 1; e <= args.episodes; ++e) {numbers.push_back(e);}
  }

  auto runner = std::make_shared<Runner>(args);
  runner->set_geometry(&geo);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(runner);
  std::thread spinner([&exec]() {exec.spin();});

  StackProcess stack;
  int stack_starts = 0;
  auto start_stack = [&]() -> bool {
      if (args.no_launch) {runner->clear_counters(); return runner->wait_ready(args.ready_timeout_s);}
      ++stack_starts;
      const std::string log = args.results_dir + "/logs/" + args.tag + "_" + args.world + "_launch_" +
        std::to_string(stack_starts) + ".log";
      std::vector<std::string> cmd = {
        "ros2", "launch", "bringup", "sim.launch.py", "world:=" + args.world,
        std::string("headless:=") + (args.gui ? "false" : "true"),
        std::string("enable_camera:=") + (args.gui ? "true" : "false"),
        "lookahead:=" + num(args.lookahead, 3), "inflation_radius:=" + num(args.inflation, 3)};
      for (const auto & extra : args.launch_args) {
        // rcl strips `key:=value` tokens from argv (they look like ROS remap rules), so the command line uses key=value
        std::string kv = extra;
        if (kv.find(":=") == std::string::npos) {
          const auto eq = kv.find('=');
          if (eq != std::string::npos) {kv.replace(eq, 1, ":=");}
        }
        cmd.push_back(kv);
      }
      std::cout << "[run_episodes] starting stack (log: " << log << ")" << std::endl;
      if (!stack.start(cmd, log)) {
        std::cerr << "[run_episodes] could not start the launch process\n";
        return false;
      }
      runner->clear_counters();
      const bool ok = runner->wait_ready(args.ready_timeout_s);
      if (!ok) {
        std::cerr << "[run_episodes] stack did not become ready within " << args.ready_timeout_s << " s\n";
      }
      // anything that died during start-up is logged separately and not charged to episode 1
      for (const auto & d : stack.poll_deaths()) {
        append_line(args.results_dir + "/crash_log.txt", wall_stamp() + " world=" + args.world +
          " STARTUP node=" + d.process + "\n  " + d.line + "\n" + d.stderr_tail);
      }
      return ok;
    };

  // optional rosbag
  pid_t bag_pid = -1;
  auto start_bag = [&]() {
      if (!args.record_bag) {return;}
      const std::string dir = args.results_dir + "/bags/" + args.tag + "_" + args.world;
      mkdirs(args.results_dir + "/bags");
      std::string rm = "rm -rf '" + dir + "'";
      if (std::system(rm.c_str()) != 0) {}
      std::vector<std::string> cmd = {"ros2", "bag", "record", "-o", dir, "/clock", "/odom", "/scan",
        "/plan", "/cmd_vel", "/obstacles", "/goal_pose", "/tf", "/tf_static", "/bumper_states"};
      std::vector<char *> cv;
      for (const auto & s : cmd) {cv.push_back(const_cast<char *>(s.c_str()));}
      cv.push_back(nullptr);
      posix_spawn_file_actions_t fa;
      posix_spawn_file_actions_init(&fa);
      posix_spawn_file_actions_addopen(&fa, 1, (args.results_dir + "/logs/" + args.tag + "_" + args.world + "_bag.log").c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      posix_spawn_file_actions_adddup2(&fa, 1, 2);
      posix_spawnattr_t attr;
      posix_spawnattr_init(&attr);
      posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSID);
      posix_spawnp(&bag_pid, cv[0], &fa, &attr, cv.data(), environ);
      posix_spawn_file_actions_destroy(&fa);
      posix_spawnattr_destroy(&attr);
    };
  auto stop_bag = [&]() {
      if (bag_pid > 0) {
        ::kill(-bag_pid, SIGINT);
        for (int i = 0; i < 100; ++i) {
          int st = 0;
          if (::waitpid(bag_pid, &st, WNOHANG) == bag_pid) {bag_pid = -1; break;}
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (bag_pid > 0) {::kill(-bag_pid, SIGKILL); int st = 0; ::waitpid(bag_pid, &st, 0); bag_pid = -1;}
      }
    };

  std::vector<EpisodeRecord> records;
  auto flush = [&]() {

      std::vector<std::string> ep_rows, diag_rows, rtf_rows;
      for (const auto & r : records) {
        std::ostringstream a, d, t;
        a << args.world << "," << r.episode << "," << r.seed << "," << (r.success ? "true" : "false") << ","
          << num(r.time_to_goal_s, 3) << "," << num(r.path_length_m, 3) << "," << r.collision_count << ","
          << num(r.min_obstacle_distance_m, 3) << "," << (r.timed_out ? "true" : "false");
        d << args.world << "," << r.episode << "," << r.seed << "," << num(r.poses.start_x, 4) << ","
          << num(r.poses.start_y, 4) << "," << num(r.poses.start_yaw, 4) << "," << num(r.poses.goal_x, 4) << ","
          << num(r.poses.goal_y, 4) << ","
          << num(std::hypot(r.poses.goal_x - r.poses.start_x, r.poses.goal_y - r.poses.start_y), 3) << ","
          << num(r.duration_s, 3) << "," << num(r.final_goal_dist_m, 3) << "," << num(r.max_stationary_s, 3)
          << "," << num(r.empty_plan_frac, 3) << "," << (r.crashed ? "true" : "false") << ","
          << r.stop_reason << "," << num(r.rtf, 3) << "," << num(r.timeout_cap_s, 1);
        ep_rows.push_back(a.str());
        diag_rows.push_back(d.str());
        if (!std::isnan(r.rtf)) {
          t << args.world << "," << r.episode << "," << num(r.rtf, 4);
          rtf_rows.push_back(t.str());
        }
      }
      merge_rows(args.results_dir + "/episodes.csv", EPISODES_HEADER, args.world, ep_rows);
      merge_rows(args.results_dir + "/episode_diagnostics.csv", DIAG_HEADER, args.world, diag_rows);
      merge_rows(args.results_dir + "/rtf_log.csv", RTF_HEADER, args.world, rtf_rows);
  };
  std::vector<double> success_times;
  bool stack_ok = start_stack();
  if (stack_ok) {start_bag();}
  int consecutive_failures = 0;

  const auto run_t0 = Clock::now();
  for (const int ep : numbers) {
    EpisodeRecord rec;
    rec.episode = ep;
    rec.seed = args.seed_base + static_cast<uint64_t>(ep);
    try {
      rec.poses = sampler.sample(rec.seed);
    } catch (const std::exception & e) {
      std::cerr << "FATAL: " << e.what() << "\n";
      exec.cancel();
      spinner.join();
      return 3;
    }
    const double cap = [&]() {
        double c = args.hard_timeout_s;
        if (success_times.size() >= 5) {
          double mean = 0;
          for (double t : success_times) {mean += t;}
          mean /= success_times.size();
          c = std::min(args.hard_timeout_s, std::max(30.0, 3.0 * mean));
        }
        return c;
      }();
    rec.timeout_cap_s = cap;

    auto mark_unavailable = [&](const std::string & why) {
        rec.crashed = true;
        rec.timed_out = true;
        rec.stop_reason = why;
        rec.min_obstacle_distance_m = std::nan("");
      };

    if (!stack_ok || (!args.no_launch && !stack.alive())) {
      std::cout << "[run_episodes] stack not running: restarting\n";
      stop_bag();
      stack.stop();
      stack_ok = start_stack();
      if (stack_ok) {start_bag();}
    }
    if (!stack_ok) {
      ++consecutive_failures;
      mark_unavailable("stack_unavailable");
      append_line(crash_log, wall_stamp() + " world=" + args.world + " episode=" + std::to_string(ep) +
        " seed=" + std::to_string(rec.seed) + " : stack could not be (re)started");
      records.push_back(rec);
      flush();
      if (consecutive_failures >= 3) {
        std::cerr << "[run_episodes] 3 consecutive stack failures: remaining episodes are recorded as crashed\n";
      }
      continue;
    }
    consecutive_failures = 0;

    // ---------- place the robot, clean the map
    runner->publish_cancel();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    bool placed = false;
    for (int attempt = 0; attempt < 3 && !placed; ++attempt) {
      if (!runner->teleport(rec.poses.start_x, rec.poses.start_y, rec.poses.start_yaw)) {continue;}
      for (int i = 0; i < 100 && !placed; ++i) {  // up to 10 s wall for the odometry to settle
        placed = runner->robot_near(rec.poses.start_x, rec.poses.start_y, 0.05);
        if (!placed) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
      }
    }
    if (!placed) {
      mark_unavailable("teleport_failed");
      append_line(crash_log, wall_stamp() + " world=" + args.world + " episode=" + std::to_string(ep) +
        " seed=" + std::to_string(rec.seed) + " : could not place the robot (set_entity_state / odom)");
      records.push_back(rec);
      flush();
      stack_ok = false;
      continue;
    }
    // let in-flight scans taken across the teleport flush out, then start a fresh map
    {
      const double t = runner->sim_now();
      while (runner->sim_now() - t < 0.6) {std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    }
    runner->reset_costmap();
    {
      const double t = runner->sim_now();
      const auto w0 = Clock::now();
      while (runner->sim_now() - t < args.warmup_s &&
        std::chrono::duration<double>(Clock::now() - w0).count() < 30.0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }

    // ---------- run
    runner->begin(rec.poses.goal_x, rec.poses.goal_y);
    const double sim_t0 = runner->sim_now();
    const auto wall_t0 = Clock::now();
    runner->publish_goal(rec.poses.goal_x, rec.poses.goal_y);

    std::string reason;
    DeathInfo death;
    auto last_progress_wall = Clock::now();
    double last_sim = sim_t0;
    auto last_death_poll = Clock::now();
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      const auto m = runner->snapshot();
      const double sim_el = runner->sim_now() - sim_t0;
      if (m.reached) {reason = "goal_reached"; break;}
      if (sim_el > cap) {reason = (cap < args.hard_timeout_s - 1e-9) ? "exceeded_3x_mean" : "hard_timeout"; break;}
      const double now_sim = runner->sim_now();
      if (now_sim > last_sim + 1e-6) {last_sim = now_sim; last_progress_wall = Clock::now();}
      if (std::chrono::duration<double>(Clock::now() - last_progress_wall).count() > 30.0) {
        reason = "sim_stalled";
        break;
      }
      if (!args.no_launch &&
        std::chrono::duration<double>(Clock::now() - last_death_poll).count() > 0.5)
      {
        last_death_poll = Clock::now();
        auto deaths = stack.poll_deaths();
        if (!deaths.empty()) {death = deaths.front(); reason = "node_crash"; break;}
        if (!stack.alive()) {reason = "launch_exited"; break;}
      }
    }

    const double sim_dur = runner->sim_now() - sim_t0;
    const double wall_dur = std::chrono::duration<double>(Clock::now() - wall_t0).count();
    const auto m = runner->snapshot();
    runner->end_measurement();
    runner->publish_cancel();

    rec.stop_reason = reason;
    rec.duration_s = (reason == "goal_reached") ? m.reach_time : sim_dur;
    rec.path_length_m = m.path_length;
    rec.collision_count = m.collisions;
    rec.min_obstacle_distance_m =
      std::isinf(m.min_obstacle_distance) ? std::nan("") : m.min_obstacle_distance;
    rec.final_goal_dist_m = m.final_dist;
    rec.max_stationary_s = m.max_stationary;
    rec.empty_plan_frac = m.plans > 0 ? static_cast<double>(m.plans_empty) / m.plans : 0.0;
    rec.rtf = (wall_dur > 0.0) ? sim_dur / wall_dur : std::nan("");
    if (reason == "goal_reached") {
      rec.success = true;
      rec.time_to_goal_s = m.reach_time;
      success_times.push_back(m.reach_time);
    } else {
      rec.timed_out = true;  // every non-success is recorded as timed_out (crashes included)
    }
    if (reason == "node_crash" || reason == "launch_exited" || reason == "sim_stalled") {
      rec.crashed = true;
      std::ostringstream cl;
      cl << wall_stamp() << " world=" << args.world << " episode=" << ep << " seed=" << rec.seed
         << " start=" << fmt_pose(rec.poses.start_x, rec.poses.start_y)
         << " goal=" << fmt_pose(rec.poses.goal_x, rec.poses.goal_y)
         << " reason=" << reason << " node=" << (death.process.empty() ? "unknown" : death.process) << "\n"
         << "  " << death.line << "\n" << death.stderr_tail;
      append_line(crash_log, cl.str());
      std::cout << "[run_episodes] CRASH in episode " << ep << ": " << reason << " " << death.process << std::endl;
      stop_bag();
      stack.stop();
      stack_ok = false;  // restarted before the next episode
    }
    if (reason == "exceeded_3x_mean") {
      append_line(limits_report, wall_stamp() + " world=" + args.world + " episode=" + std::to_string(ep) +
        " seed=" + std::to_string(rec.seed) + " start=" + fmt_pose(rec.poses.start_x, rec.poses.start_y) +
        " goal=" + fmt_pose(rec.poses.goal_x, rec.poses.goal_y) +
        " : exceeded " + num(cap, 1) + " s (3x mean success time); marked timed_out");
    }
    if (!std::isnan(rec.rtf) && rec.rtf < 0.5) {
      append_line(limits_report, wall_stamp() + " world=" + args.world + " episode=" + std::to_string(ep) +
        " : low real-time factor " + num(rec.rtf, 3) + (rec.rtf < 0.3 ? " (< 0.3)" : " (< 0.5)"));
    }

    std::cout << "[run_episodes] " << args.world << " ep " << std::setw(2) << ep << "/" << numbers.size()
              << " seed " << rec.seed << " start " << fmt_pose(rec.poses.start_x, rec.poses.start_y)
              << " goal " << fmt_pose(rec.poses.goal_x, rec.poses.goal_y) << " -> "
              << (rec.success ? "SUCCESS" : "FAIL") << " [" << reason << "] t=" << num(rec.duration_s, 1)
              << "s path=" << num(rec.path_length_m, 2) << "m coll=" << rec.collision_count
              << " mind=" << num(rec.min_obstacle_distance_m, 2) << " rtf=" << num(rec.rtf, 2) << std::endl;
    records.push_back(rec);

    flush();
  }

  stop_bag();
  runner->publish_cancel();
  if (!args.no_launch) {stack.stop();}
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();

  int ok = 0, coll = 0, to = 0, crashed = 0;
  for (const auto & r : records) {
    ok += r.success;
    coll += r.collision_count;
    to += r.timed_out;
    crashed += r.crashed;
  }
  const double wall_total = std::chrono::duration<double>(Clock::now() - run_t0).count();
  std::cout << "[run_episodes] DONE world=" << args.world << " episodes=" << records.size() << " success=" << ok
            << " collisions=" << coll << " timed_out=" << to << " crashed=" << crashed
            << " wall=" << num(wall_total / 60.0, 1) << " min" << std::endl;
  append_line(args.results_dir + "/logs/wallclock.txt",
    wall_stamp() + " tag=" + args.tag + " world=" + args.world + " episodes=" + std::to_string(records.size()) +
    " wall_minutes=" + num(wall_total / 60.0, 2));
  return 0;
}
