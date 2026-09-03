#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <atomic>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <kino/kinodynamic_astar.h>
#include <string>
#include <thread>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber gridmap_sub_, odom_sub_, target_sub_, triger_sub_, land_triger_sub_;
  ros::Timer plan_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_;
  ros::Publisher kino_path_pub_;
  ros::Publisher kino_minco_path_pub_;

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  fast_planner::KinodynamicAstar::Ptr kinoAstarPtr_;

  bool kinoAstarInitialized_ = false;

  // NOTE planning or fake target
  bool fake_ = false;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::ReplanState replanStateMsg_;
  ros::Publisher gridmap_pub_, inflate_gridmap_pub_;
  quadrotor_msgs::OccMap3d occmap_msg_;

  double tracking_dur_, tracking_dist_, tolerance_d_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  nav_msgs::Odometry odom_msg_, target_msg_;
  quadrotor_msgs::OccMap3d map_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  // NOTE publish Kinodynamic A* path on an independent ROS topic
  void pub_kino_path(const std::vector<Eigen::Vector3d>& path) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();
    path_msg.poses.reserve(path.size());

    for (const auto& p : path) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();

      // nav_msgs::Path 主要用于显示位置轨迹。
      // 设置合法单位四元数，避免 RViz 出现无效姿态警告。
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path_msg.poses.push_back(pose);
    }

    kino_path_pub_.publish(path_msg);
  }

  // NOTE publish Kino->SFC->visibility-aware MINCO result on an independent ROS topic
  void pub_kino_minco_path(const Trajectory& traj) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    const double duration = traj.getTotalDuration();
    const double sample_dt = 0.02;

    for (double t = 0.0; t <= duration; t += sample_dt) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;

      const Eigen::Vector3d p = traj.getPos(t);
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path_msg.poses.push_back(pose);
    }

    // 确保轨迹终点一定被发布，即使 duration 不是 sample_dt 的整数倍。
    if (duration > 0.0) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;

      const Eigen::Vector3d p = traj.getPos(duration);
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path_msg.poses.push_back(pose);
    }

    kino_minco_path_pub_.publish(path_msg);
  }

  void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
  }
  void pub_traj(const Trajectory& traj, const double& yaw, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_pub_.publish(traj_msg);
  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    triger_received_ = true;
  }

  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_msg_ = *msgPtr;
    target_received_ = true;
    target_lock_.clear();
  }

  void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    while (gridmap_lock_.test_and_set())
      ;
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  // ============================================================
  // Kino 主规划链：
  //
  //   No-A* Visibility Selector
  //          -> Kinodynamic A*
  //          -> SFC
  //          -> Visibility-aware MINCO
  //
  // 返回 true：
  //   已经生成一条通过碰撞检查、可以交给 pub_traj() 的轨迹。
  //
  // 返回 false：
  //   主 callback 会立即回退到原 Elastic Tracker 前端。
  // ============================================================
  bool generate_kino_primary_trajectory(
      const Eigen::MatrixXd& iniState,
      const Eigen::Vector3d& target_v,
      const std::vector<Eigen::Vector3d>& target_predict,
      const ros::Time& replan_stamp,
      Trajectory& traj_out,
      std::string& fail_reason) {
    fail_reason.clear();

    if (!kinoAstarInitialized_) {
      fail_reason = "Kinodynamic A* is not initialized.";
      return false;
    }

    if (target_predict.size() < 2) {
      fail_reason = "target prediction has fewer than 2 points.";
      return false;
    }

    const Eigen::Vector3d start_pos = iniState.col(0);
    const Eigen::Vector3d start_vel = iniState.col(1);
    const Eigen::Vector3d start_acc = iniState.col(2);

    // ----------------------------------------------------------
    // 1. No-A* visibility waypoint selector
    // ----------------------------------------------------------
    std::vector<Eigen::Vector3d> selected_waypoints;

    if (!envPtr_->selectVisibleWaypoints(
            start_pos,
            target_predict,
            selected_waypoints)) {
      fail_reason = "visibility waypoint selector failed.";
      return false;
    }

    if (selected_waypoints.size() < 2) {
      fail_reason = "not enough selected visibility waypoints.";
      return false;
    }

    visPtr_->visualize_pointcloud(
        selected_waypoints,
        "kino_way_pts_no_astar");

    // ----------------------------------------------------------
    // 2. 和原 Elastic tracking 保持同样的预测序列长度关系
    //
    // 原流程会：
    //   target_predcit.pop_back();
    //   way_pts.pop_back();
    // ----------------------------------------------------------
    std::vector<Eigen::Vector3d> visible_targets =
        target_predict;

    std::vector<Eigen::Vector3d> visible_seeds =
        selected_waypoints;

    visible_targets.pop_back();
    visible_seeds.pop_back();

    if (visible_targets.empty() ||
        visible_seeds.empty() ||
        visible_targets.size() != visible_seeds.size()) {
      fail_reason = "invalid visibility target/seed sequence.";
      return false;
    }

    // ----------------------------------------------------------
    // 3. 生成 visibility regions
    //
    // visible_pair() 可能会修正 seed，因此 Kino 的真正终点
    // 必须使用修正后的 visible_seeds.back()。
    // ----------------------------------------------------------
    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> visible_thetas;

    envPtr_->generate_visible_regions(
        visible_targets,
        visible_seeds,
        visible_ps,
        visible_thetas);

    if (visible_ps.empty() ||
        visible_thetas.empty() ||
        visible_ps.size() != visible_targets.size() ||
        visible_thetas.size() != visible_targets.size()) {
      fail_reason = "visible region generation failed.";
      return false;
    }

    visPtr_->visualize_pointcloud(
        visible_ps,
        "kino_visible_ps");

    visPtr_->visualize_fan_shape_meshes(
        visible_targets,
        visible_ps,
        visible_thetas,
        "kino_visible_region");

    const Eigen::Vector3d goal_pos =
        visible_seeds.back();

    const Eigen::Vector3d goal_vel =
        target_v;

    // ----------------------------------------------------------
    // 4. Kinodynamic A*
    // ----------------------------------------------------------
    kinoAstarPtr_->reset();

    int kino_status =
        kinoAstarPtr_->search(
            start_pos,
            start_vel,
            start_acc,
            goal_pos,
            goal_vel,
            true);

    if (kino_status ==
        fast_planner::KinodynamicAstar::NO_PATH) {
      kinoAstarPtr_->reset();

      kino_status =
          kinoAstarPtr_->search(
              start_pos,
              start_vel,
              start_acc,
              goal_pos,
              goal_vel,
              false);
    }

    std::vector<fast_planner::PathNodePtr> visited_nodes =
        kinoAstarPtr_->getVisitedNodes();

    // 当前正式接管阶段仍要求真正到达 visibility waypoint。
    // REACH_HORIZON 视为本周期主规划失败，交给 Elastic fallback。
    if (kino_status !=
        fast_planner::KinodynamicAstar::REACH_END) {
      if (kino_status ==
          fast_planner::KinodynamicAstar::REACH_HORIZON) {
        fail_reason = "Kinodynamic A* reached horizon before visibility goal.";
      } else {
        fail_reason = "Kinodynamic A* found no path.";
      }
      return false;
    }

    std::vector<Eigen::Vector3d> kino_path =
        kinoAstarPtr_->getKinoTraj(0.1);

    if (kino_path.empty()) {
      fail_reason = "Kinodynamic A* returned an empty path.";
      return false;
    }

    // getKinoTraj() 最后一个采样点可能略早于精确终点，
    // 补入精确 goal 以确保 SFC 覆盖 MINCO 的终端状态。
    if ((kino_path.back() - goal_pos).norm() > 1e-3) {
      kino_path.push_back(goal_pos);
    }

    pub_kino_path(kino_path);

    // ----------------------------------------------------------
    // 5. Kino path -> SFC
    // ----------------------------------------------------------
    std::vector<Eigen::MatrixXd> kino_hPolys;
    std::vector<
        std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        kino_keyPts;

    envPtr_->generateSFC(
        kino_path,
        2.0,
        kino_hPolys,
        kino_keyPts);

    if (kino_hPolys.empty()) {
      fail_reason = "Kino SFC generation returned an empty corridor.";
      return false;
    }

    // 主规划器成功时，把当前真正使用的安全走廊显示出来。
    envPtr_->visCorridor(kino_hPolys);
    visPtr_->visualize_pairline(
        kino_keyPts,
        "kino_keyPts");

    // ----------------------------------------------------------
    // 6. SFC -> Visibility-aware MINCO
    // ----------------------------------------------------------
    Eigen::MatrixXd finState;
    finState.setZero(3, 3);
    finState.col(0) = goal_pos;
    finState.col(1) = target_v;

    Trajectory kino_traj;

    const bool minco_success =
        trajOptPtr_->generate_traj(
            iniState,
            finState,
            visible_targets,
            visible_ps,
            visible_thetas,
            kino_hPolys,
            kino_traj);

    if (!minco_success) {
      fail_reason = "visibility-aware MINCO failed.";
      return false;
    }

    const double duration =
        kino_traj.getTotalDuration();

    if (duration <= 0.0) {
      fail_reason = "MINCO returned a non-positive trajectory duration.";
      return false;
    }

    // ----------------------------------------------------------
    // 7. 在真正交给无人机前，再做一次完整地图碰撞检查
    //
    // 同时拒绝当前 rolling map 之外的点。
    // ----------------------------------------------------------
    Eigen::Vector3d collision_point =
        Eigen::Vector3d::Zero();

    for (double t = 0.0;
         t <= duration;
         t += 0.01) {
      const Eigen::Vector3d p =
          kino_traj.getPos(t);

      if (!gridmapPtr_->isInMap(p) ||
          gridmapPtr_->isOccupied(p)) {
        collision_point = p;

        fail_reason =
            "Kino MINCO trajectory is outside map or in collision at [" +
            std::to_string(p.x()) + ", " +
            std::to_string(p.y()) + ", " +
            std::to_string(p.z()) + "].";

        return false;
      }
    }

    // 独立绿色 Path 继续保留，方便确认“实际执行轨迹”
    // 与 Kino MINCO 调试轨迹一致。
    pub_kino_minco_path(kino_traj);

    // 到这里才允许交给主 callback 发布。
    traj_out = kino_traj;

    ROS_WARN_STREAM(
        "[KINO PRIMARY] trajectory ready"
        << "\nvisited nodes  : "
        << visited_nodes.size()
        << "\nkino path pts  : "
        << kino_path.size()
        << "\ncorridor count : "
        << kino_hPolys.size()
        << "\nvisible points : "
        << visible_ps.size()
        << "\nduration       : "
        << duration
        << " s"
        << "\npiece num      : "
        << kino_traj.getPieceNum());

    return true;
  }

  // NOTE main callback
  void plan_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);

    
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();
    prePtr_->setMap(*gridmapPtr_);

    if (!kinoAstarInitialized_)
    {
      kinoAstarPtr_->init();
      kinoAstarInitialized_ = true;

      ROS_INFO("[KinodynamicAstar] initialization finished.");
    }

    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                              odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z);
    if (!triger_received_) {
      return;
    }
    if (!target_received_) {
      return;
    }
    // NOTE obtain state of target
    while (target_lock_.test_and_set())
      ;
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE just for landing on the car!
    if (land_triger_received_) {
      if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2)) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing point
      target_p = target_p + target_q * land_p_;
      wait_hover_ = false;
    } else {
      target_p.z() += 1.0;
      // NOTE determin whether to replan
      Eigen::Vector3d dp = target_p - odom_p;
      // std::cout << "dist : " << dp.norm() << std::endl;
      double desired_yaw = std::atan2(dp.y(), dp.x());
      Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
      if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ &&
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
          std::fabs(desired_yaw - now_yaw) < 0.5) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[planner] HOVERING...");
        replanStateMsg_.state = -1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      } else {
        wait_hover_ = false;
      }
    }



    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    // ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);
    // ros::Time t_stop = ros::Time::now();
    // std::cout << "predict costs: " << (t_stop - t_start).toSec() * 1e3 << "ms" << std::endl;
    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // ============================================================
    // PRIMARY / FALLBACK planning
    //
    // Normal tracking:
    //
    //   PRIMARY:
    //     No-A* Selector -> Kino -> SFC -> Visibility MINCO
    //
    //   FALLBACK:
    //     原 Elastic findVisiblePath -> pts2path -> SFC -> MINCO
    //
    // Landing:
    //   暂时保持原 Elastic landing 流程，不在本阶段修改。
    // ============================================================
    Eigen::Vector3d p_start = iniState.col(0);

    Trajectory traj;

    bool kino_primary_used = false;
    bool elastic_fallback_used = false;
    bool elastic_landing_used = false;

    // predict() 失败时保持失败状态，直接进入后面的通用安全处理。
    if (generate_new_traj_success) {
      if (land_triger_received_) {
        // ========================================================
        // Landing：保持原 Elastic Tracker 实现
        // ========================================================
        elastic_landing_used = true;

        std::vector<Eigen::Vector3d> path;

        generate_new_traj_success =
            envPtr_->short_astar(
                p_start,
                target_p,
                path);

        if (generate_new_traj_success) {
          for (const auto& p : target_predcit) {
            path.push_back(p);
          }

          if (path.size() < 2) {
            generate_new_traj_success = false;
          }
        }

        if (generate_new_traj_success) {
          visPtr_->visualize_path(
              path,
              "astar");

          std::vector<Eigen::MatrixXd> hPolys;
          std::vector<
              std::pair<Eigen::Vector3d, Eigen::Vector3d>>
              keyPts;

          envPtr_->generateSFC(
              path,
              2.0,
              hPolys,
              keyPts);

          if (hPolys.empty()) {
            generate_new_traj_success = false;
          } else {
            envPtr_->visCorridor(hPolys);
            visPtr_->visualize_pairline(
                keyPts,
                "keyPts");

            Eigen::MatrixXd finState;
            finState.setZero(3, 3);
            finState.col(0) =
                target_predcit.back();

            generate_new_traj_success =
                trajOptPtr_->generate_traj(
                    iniState,
                    finState,
                    target_predcit,
                    hPolys,
                    traj);
          }
        }
      }
      else {
        // ========================================================
        // 1. Kino 主规划器
        // ========================================================
        std::string kino_fail_reason;

        kino_primary_used =
            generate_kino_primary_trajectory(
                iniState,
                target_v,
                target_predcit,
                replan_stamp,
                traj,
                kino_fail_reason);

        if (kino_primary_used) {
          generate_new_traj_success = true;
        }
        else {
          // ======================================================
          // 2. Kino 失败 -> 原 Elastic Tracker fallback
          // ======================================================
          ROS_WARN_STREAM(
              "[KINO PRIMARY] FAILED"
              << "\nreason: "
              << kino_fail_reason
              << "\n[ELASTIC FALLBACK] attempting original planner.");

          elastic_fallback_used = true;

          std::vector<Eigen::Vector3d> path;
          std::vector<Eigen::Vector3d> way_pts;

          generate_new_traj_success =
              envPtr_->findVisiblePath(
                  p_start,
                  target_predcit,
                  way_pts,
                  path);

          if (generate_new_traj_success) {
            if (target_predcit.size() < 2 ||
                way_pts.size() < 2) {
              generate_new_traj_success = false;
            }
          }

          if (generate_new_traj_success) {
            visPtr_->visualize_path(
                path,
                "astar");

            // 和原 Elastic Tracker 完全相同：
            // 删除最后一个 prediction / waypoint 后生成 visibility region。
            target_predcit.pop_back();
            way_pts.pop_back();

            std::vector<Eigen::Vector3d> visible_ps;
            std::vector<double> thetas;

            envPtr_->generate_visible_regions(
                target_predcit,
                way_pts,
                visible_ps,
                thetas);

            visPtr_->visualize_pointcloud(
                visible_ps,
                "visible_ps");

            visPtr_->visualize_fan_shape_meshes(
                target_predcit,
                visible_ps,
                thetas,
                "visible_region");

            visPtr_->visualize_pointcloud(
                way_pts,
                "way_pts");

            // 原 Elastic 的第二层几何连接。
            // 如果直线不可行，pts2path() 内部仍可能调用 short_astar()。
            way_pts.insert(
                way_pts.begin(),
                p_start);

            envPtr_->pts2path(
                way_pts,
                path);

            if (path.size() < 2) {
              generate_new_traj_success = false;
            }

            if (generate_new_traj_success) {
              std::vector<Eigen::MatrixXd> hPolys;
              std::vector<
                  std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                  keyPts;

              envPtr_->generateSFC(
                  path,
                  2.0,
                  hPolys,
                  keyPts);

              if (hPolys.empty()) {
                generate_new_traj_success = false;
              }
              else {
                envPtr_->visCorridor(
                    hPolys);

                visPtr_->visualize_pairline(
                    keyPts,
                    "keyPts");

                Eigen::MatrixXd finState;
                finState.setZero(3, 3);
                finState.col(0) =
                    path.back();
                finState.col(1) =
                    target_v;

                generate_new_traj_success =
                    trajOptPtr_->generate_traj(
                        iniState,
                        finState,
                        target_predcit,
                        visible_ps,
                        thetas,
                        hPolys,
                        traj);
              }
            }
          }
        }
      }
    }

    // 当前实际被选择的轨迹统一显示在原 "traj" topic。
    if (generate_new_traj_success) {
      visPtr_->visualize_traj(
          traj,
          "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      if (kino_primary_used) {
        ROS_WARN("[KINO PRIMARY] REPLAN SUCCESS - drone executes Kino trajectory");
      }
      else if (elastic_fallback_used) {
        ROS_WARN("[ELASTIC FALLBACK] REPLAN SUCCESS - drone executes fallback trajectory");
      }
      else if (elastic_landing_used) {
        ROS_WARN("[ELASTIC LANDING] REPLAN SUCCESS");
      }
      else {
        ROS_WARN("[planner] REPLAN SUCCESS");
      }
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
      // NOTE : if the drone is going to unknown areas, watch that direction
      // Eigen::Vector3d un_known_p = traj.getPos(1.0);
      // if (gridmapPtr_->isUnKnown(un_known_p)) {
      //   dp = un_known_p - odom_p;
      // }
      double yaw = std::atan2(dp.y(), dp.x());
      if (land_triger_received_) {
        yaw = 2 * std::atan2(target_q.z(), target_q.w());
      }
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void fake_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {
      return;
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    if (delta.norm() < 15) {
      local_goal = goal_;
    } else {
      local_goal = delta.normalized() * 15 + odom_p;
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // NOTE determin whether to replan
    bool no_need_replan = false;
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() - (ros::Time::now() - replan_stamp_).toSec();
      bool new_goal = (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration())).norm() > tracking_dist_;
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0 ? traj_poly_.getTotalDuration() : 1.0;
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;
        }
      }
    }
    // NOTE determin whether to pub hover
    if ((goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {
      if (!wait_hover_) {
        pub_hover_p(odom_p, ros::Time::now());
        wait_hover_ = true;
      }
      ROS_WARN("[planner] HOVERING...");
      replanStateMsg_.state = -1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else {
      wait_hover_ = false;
    }
    if (no_need_replan) {
      return;
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_poly_.getPos(replan_t);
      iniState.col(1) = traj_poly_.getVel(replan_t);
      iniState.col(2) = traj_poly_.getAcc(replan_t);
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      // return;
      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    inflate_gridmap_pub_.publish(replanStateMsg_.occmap);
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);

    iniState = Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    // std::cout << "target_p: " << target_p.transpose() << std::endl;
    // std::cout << "target_v: " << target_v.transpose() << std::endl;

    // visualize the target and the drone velocity
    visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1), "drone_vel");
    visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    if (gridmapPtr_->isOccupied(target_p)) {
      std::cout << "target is invalid!" << std::endl;
      assert(false);
    }
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);

    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE generate visible regions
      target_predcit.pop_back();
      way_pts.pop_back();
      envPtr_->generate_visible_regions(target_predcit, way_pts,
                                        visible_ps, thetas);
      visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // TODO change the final state
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predcit[i], way_pts[i]);
      }
      visPtr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      envPtr_->pts2path(way_pts, path);
      visPtr_->visualize_path(path, "corridor_path");
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;

      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                             target_predcit, visible_ps, thetas,
                                                             hPolys, traj);
      visPtr_->visualize_traj(traj, "traj");
    }
    if (!generate_new_traj_success) {
      return;
      // assert(false);
    }
    // check
    bool valid = true;
    std::vector<Eigen::Vector3d> check_pts, invalid_pts;
    double t0 = (ros::Time::now() - replan_stamp).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double check_dur = 1.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.1) {
      Eigen::Vector3d p = traj.getPos(t);
      check_pts.push_back(p);
      if (gridmapPtr_->isOccupied(p)) {
        invalid_pts.push_back(p);
      }
    }
    visPtr_->visualize_path(invalid_pts, "invalid_pts");
    visPtr_->visualize_path(check_pts, "check_pts");
    valid = validcheck(traj, replan_stamp);
    if (!valid) {
      std::cout << "invalid!" << std::endl;
    }
  }

  bool validcheck(const Trajectory& traj, const ros::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    int plan_hz = 10;
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("tolerance_d", tolerance_d_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    kinoAstarPtr_ = std::make_shared<fast_planner::KinodynamicAstar>();
    kinoAstarPtr_->setParam(nh);
    kinoAstarPtr_->setEnvironment(gridmapPtr_);
    envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    prePtr_ = std::make_shared<prediction::Predict>(nh);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);
    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 1);
    replanState_pub_ = nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);
    kino_path_pub_ = nh.advertise<nav_msgs::Path>("kino_astar_path", 10);
    kino_minco_path_pub_ = nh.advertise<nav_msgs::Path>("kino_minco_traj", 10);

    if (debug_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::debug_timer_callback, this);
      // TODO read debug data from files
      wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") + "/../../../debug/replan_state.bin");
      inflate_gridmap_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
      gridmapPtr_->from_msg(replanStateMsg_.occmap);
      prePtr_->setMap(*gridmapPtr_);
      std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    } else if (fake_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::fake_timer_callback, this);
    } else {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::plan_timer_callback, this);
    }
    gridmap_sub_ = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, &Nodelet::gridmap_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    target_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 10, &Nodelet::target_callback, this, ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("land_triger", 10, &Nodelet::land_triger_callback, this, ros::TransportHints().tcpNoDelay());
    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);