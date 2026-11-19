// Dynamic task allocation for multi-UAV semantic inspection
// (ICUAS 2025, Sec. IV-C).
//
// Tasks are semantic objects reported on /<ns>/previous_semantic_points.
// Two strategies, selected with ~strategy:
//
//  * "auction"  Sequential single-item auction (Schneider et al., TAROS'15).
//               After the exploration budget ~exploration_time_budget, all
//               unassigned tasks are auctioned. Every round each robot bids
//               its marginal cost for every open task:
//                 B_ij = travel_time(end of i's queue -> task j)
//                        + inspection_time + workload_weight * queue_time_i
//               Bids exceeding a robot's remaining battery time are dropped.
//               The globally lowest bid wins; repeat until no task is left.
//               Tasks detected later are auctioned the same way, so the task
//               set T is updated dynamically.
//
//  * "role"     Fixed roles from ~roles (explorer / inspector). Each task is
//               assigned greedily, immediately on detection, to the closest
//               inspector (load-balanced when there are more tasks than
//               inspectors). An explorer takes over a task only if no
//               inspector is within ~comm_range of it.
//
// A robot with tasks has its exploration planner paused; the inspection
// planner (gbplanner, /<ns>/semantic_location) reports completion on
// /<ns>/done_inspection, after which the next task is sent. A robot whose
// queue is empty resumes exploration.

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <planner_semantic_msgs/SemanticPoint.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

struct Task {
  int id;
  geometry_msgs::Point point;
  int type;
  std::string reported_by;
  std::string assigned_to;  // empty = unassigned
  bool done = false;
};

double dist(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}

}  // namespace

class DynamicTaskAllocator {
 public:
  explicit DynamicTaskAllocator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("strategy", strategy_, "auction");
    std::vector<std::string> robots;
    pnh_.param("robots", robots, std::vector<std::string>{"red", "blue"});
    pnh_.param("exploration_time_budget", exploration_budget_, 500.0);
    pnh_.param("task_merge_radius", merge_radius_, 6.0);
    pnh_.param("inspection_time_per_task", inspection_time_, 30.0);
    pnh_.param("v_max", v_max_, 1.0);
    pnh_.param("workload_weight", workload_weight_, 1.0);
    pnh_.param("comm_range", comm_range_, 50.0);
    pnh_.param("battery_time_budget", battery_budget_, 1e9);
    pnh_.param<std::string>("world_frame", world_frame_, "world");

    if (strategy_ != "auction" && strategy_ != "role") {
      ROS_WARN("Unknown strategy '%s', using 'auction'.", strategy_.c_str());
      strategy_ = "auction";
    }

    for (const auto& name : robots) {
      auto r = std::make_shared<Robot>();
      r->name = name;
      std::string role;
      pnh_.param<std::string>("roles/" + name, role,
                              strategy_ == "role" ? "explorer" : "any");
      r->role = role;
      const std::string ns = "/" + name;
      r->toggle = nh_.serviceClient<std_srvs::SetBool>(ns + "/gbplanner/toggle_planning");
      r->stop = nh_.serviceClient<std_srvs::Trigger>(ns + "/planner_control_interface/std_srvs/stop");
      r->automatic = nh_.serviceClient<std_srvs::Trigger>(ns + "/planner_control_interface/std_srvs/automatic_planning");
      r->task_pub = nh_.advertise<planner_semantic_msgs::SemanticPoint>(ns + "/semantic_location", 10);
      r->odom_sub = nh_.subscribe<nav_msgs::Odometry>(
          ns + "/odometry", 1,
          [r](const nav_msgs::Odometry::ConstPtr& m) {
            r->pose = m->pose.pose.position;
            r->has_pose = true;
          });
      r->done_sub = nh_.subscribe<std_msgs::Bool>(
          ns + "/done_inspection", 1,
          [this, r](const std_msgs::Bool::ConstPtr& m) {
            if (m->data) onInspectionDone(*r);
          });
      r->sem_sub = nh_.subscribe<planner_semantic_msgs::SemanticPoint>(
          ns + "/previous_semantic_points", 50,
          [this, name](const planner_semantic_msgs::SemanticPoint::ConstPtr& m) {
            onDetection(*m, name);
          });
      robots_.push_back(r);
      ROS_INFO("[TA] robot %s role=%s", name.c_str(), role.c_str());
    }

    start_time_ = ros::Time::now();
    timer_ = nh_.createTimer(ros::Duration(1.0), &DynamicTaskAllocator::tick, this);
    ROS_INFO("[TA] strategy=%s, exploration budget %.0f s, %zu robots",
             strategy_.c_str(), exploration_budget_, robots_.size());
  }

 private:
  struct Robot {
    std::string name, role;
    geometry_msgs::Point pose;
    bool has_pose = false;
    std::deque<int> queue;  // task ids; front() is being inspected
    bool inspecting = false;
    bool exploring = true;
    ros::ServiceClient toggle, stop, automatic;
    ros::Publisher task_pub;
    ros::Subscriber odom_sub, done_sub, sem_sub;
  };

  // ---------------------------------------------------------------- events
  void onDetection(const planner_semantic_msgs::SemanticPoint& m,
                   const std::string& reporter) {
    geometry_msgs::Point p;
    p.x = m.point.x;
    p.y = m.point.y;
    p.z = m.point.z;
    for (const auto& t : tasks_)
      if (dist(t.point, p) < merge_radius_) return;  // already known
    Task t;
    t.id = static_cast<int>(tasks_.size());
    t.point = p;
    t.type = m.type.value;
    t.reported_by = reporter;
    tasks_.push_back(t);
    ROS_INFO("[TA] new task %d (type %d) at (%.1f, %.1f, %.1f) from %s", t.id,
             t.type, p.x, p.y, p.z, reporter.c_str());
    if (strategy_ == "role") assignRoleBased(tasks_.back());
  }

  void onInspectionDone(Robot& r) {
    if (!r.inspecting || r.queue.empty()) return;
    const int id = r.queue.front();
    tasks_[id].done = true;
    r.queue.pop_front();
    r.inspecting = false;
    ROS_INFO("[TA] %s finished task %d (%d/%zu done)", r.name.c_str(), id,
             numDone(), tasks_.size());
    dispatch(r);
  }

  void tick(const ros::TimerEvent&) {
    const double elapsed = (ros::Time::now() - start_time_).toSec();
    if (strategy_ == "auction") {
      if (elapsed >= exploration_budget_) {
        if (!auction_opened_) {
          auction_opened_ = true;
          ROS_INFO("[TA] exploration budget reached (%.0f s): auction open.",
                   elapsed);
        }
        runAuction();
      }
    }
    for (auto& r : robots_) dispatch(*r);
  }

  // ---------------------------------------------------------- strategies
  // Time for robot r to finish its current queue, and where it ends up.
  double queueTime(const Robot& r, geometry_msgs::Point& end) const {
    end = r.pose;
    double t = 0.0;
    for (int id : r.queue) {
      t += dist(end, tasks_[id].point) / v_max_ + inspection_time_;
      end = tasks_[id].point;
    }
    return t;
  }

  double remainingBattery() const {
    return battery_budget_ - (ros::Time::now() - start_time_).toSec();
  }

  void runAuction() {
    std::vector<int> open;
    for (const auto& t : tasks_)
      if (!t.done && t.assigned_to.empty()) open.push_back(t.id);
    if (open.empty()) return;

    while (!open.empty()) {
      double best_bid = std::numeric_limits<double>::infinity();
      Robot* winner = nullptr;
      int best_task = -1;
      for (auto& r : robots_) {
        if (!r->has_pose) continue;
        geometry_msgs::Point end;
        const double qt = queueTime(*r, end);
        for (int id : open) {
          const double marginal =
              dist(end, tasks_[id].point) / v_max_ + inspection_time_;
          if (qt + marginal > remainingBattery()) continue;  // can't afford
          const double bid = marginal + workload_weight_ * qt;
          if (bid < best_bid) best_bid = bid, winner = r.get(), best_task = id;
        }
      }
      if (!winner) {
        ROS_WARN_THROTTLE(10.0, "[TA] %zu task(s) not affordable by any robot.",
                          open.size());
        return;
      }
      tasks_[best_task].assigned_to = winner->name;
      winner->queue.push_back(best_task);
      open.erase(std::find(open.begin(), open.end(), best_task));
      ROS_INFO("[TA] auction: task %d -> %s (bid %.1f s)", best_task,
               winner->name.c_str(), best_bid);
    }
  }

  void assignRoleBased(Task& t) {
    auto pick = [&](const std::string& role, bool require_range) -> Robot* {
      Robot* best = nullptr;
      double best_cost = std::numeric_limits<double>::infinity();
      for (auto& r : robots_) {
        if (r->role != role || !r->has_pose) continue;
        const double d = dist(r->pose, t.point);
        if (require_range && d > comm_range_) continue;
        // Greedy: closest robot, ties broken by fewer queued tasks.
        const double cost = d + 1e3 * r->queue.size();
        if (cost < best_cost) best_cost = cost, best = r.get();
      }
      return best;
    };
    Robot* r = pick("inspector", true);
    if (!r) r = pick("explorer", false);   // inspector out of range: take over
    if (!r) r = pick("inspector", false);  // no explorer either
    if (!r) {
      ROS_WARN("[TA] no robot available for task %d yet.", t.id);
      return;
    }
    t.assigned_to = r->name;
    r->queue.push_back(t.id);
    ROS_INFO("[TA] role-based: task %d -> %s (%s)", t.id, r->name.c_str(),
             r->role.c_str());
  }

  // ------------------------------------------------------------ dispatch
  void dispatch(Robot& r) {
    if (r.inspecting) return;
    if (!r.queue.empty()) {
      if (r.exploring) setExploration(r, false);
      const Task& t = tasks_[r.queue.front()];
      planner_semantic_msgs::SemanticPoint msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = world_frame_;
      msg.type.value = t.type;
      msg.point.x = t.point.x;
      msg.point.y = t.point.y;
      msg.point.z = t.point.z;
      r.task_pub.publish(msg);
      r.inspecting = true;
      ROS_INFO("[TA] %s: inspecting task %d at (%.1f, %.1f, %.1f)",
               r.name.c_str(), t.id, t.point.x, t.point.y, t.point.z);
      return;
    }
    // Nothing to inspect. Explorers always explore; in auction mode every
    // robot explores until the budget is spent and again once idle.
    if (!r.exploring && (r.role != "inspector" || strategy_ == "auction"))
      setExploration(r, true);
  }

  void setExploration(Robot& r, bool on) {
    std_srvs::SetBool tog;
    tog.request.data = on;
    if (!r.toggle.call(tog)) {
      ROS_WARN_THROTTLE(5.0, "[TA] %s: toggle_planning unavailable.",
                        r.name.c_str());
      return;
    }
    std_srvs::Trigger trig;
    r.stop.call(trig);
    if (on) r.automatic.call(trig);
    r.exploring = on;
    ROS_INFO("[TA] %s: exploration %s", r.name.c_str(), on ? "ON" : "OFF");
  }

  int numDone() const {
    return static_cast<int>(std::count_if(tasks_.begin(), tasks_.end(),
                                          [](const Task& t) { return t.done; }));
  }

  ros::NodeHandle nh_, pnh_;
  std::string strategy_, world_frame_;
  double exploration_budget_, merge_radius_, inspection_time_, v_max_,
      workload_weight_, comm_range_, battery_budget_;
  std::vector<std::shared_ptr<Robot>> robots_;
  std::vector<Task> tasks_;
  bool auction_opened_ = false;
  ros::Time start_time_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_auction");
  ros::NodeHandle nh, pnh("~");
  // Wait for /clock under use_sim_time so start_time_ is meaningful.
  while (ros::ok() && ros::Time::now().isZero()) {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }
  DynamicTaskAllocator allocator(nh, pnh);
  ros::spin();
  return 0;
}
