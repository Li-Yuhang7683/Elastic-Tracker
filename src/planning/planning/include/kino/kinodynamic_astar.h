#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

#include <boost/functional/hash.hpp>

#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Elastic Tracker 自己的占据栅格地图
#include <mapping/mapping.h>

namespace fast_planner
{

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

/**
 * @brief Kinodynamic A* 中的搜索节点
 *
 * state:
 *   [px, py, pz, vx, vy, vz]
 *
 * input:
 *   从 parent 到当前节点所使用的加速度输入
 */
class PathNode
{
public:
  Eigen::Vector3i index;

  Eigen::Matrix<double, 6, 1> state;

  double g_score;
  double f_score;

  Eigen::Vector3d input;

  double duration;

  // 动态环境搜索时使用
  double time;
  int time_idx;

  PathNode *parent;

  char node_state;

  PathNode()
  {
    parent = NULL;
    node_state = NOT_EXPAND;
  }

  ~PathNode() {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef PathNode *PathNodePtr;


/**
 * @brief open_set_ 的比较函数
 *
 * priority_queue 默认大值优先，
 * 这里通过反向比较，让 f_score 小的节点优先。
 */
class NodeComparator
{
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};


/**
 * @brief Eigen 向量的哈希函数
 *
 * 使 Eigen::Vector3i / Vector4i 可以作为 unordered_map 的 key。
 */
template <typename T>
struct matrix_hash
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;

    for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i)
    {
      auto elem = *(matrix.data() + i);

      seed ^= std::hash<typename T::Scalar>()(elem)
              + 0x9e3779b9
              + (seed << 6)
              + (seed >> 2);
    }

    return seed;
  }
};


/**
 * @brief 搜索节点哈希表
 *
 * 静态环境：
 *   (x,y,z) -> PathNode
 *
 * 动态环境：
 *   (x,y,z,t) -> PathNode
 */
class NodeHashTable
{
private:
  std::unordered_map<
      Eigen::Vector3i,
      PathNodePtr,
      matrix_hash<Eigen::Vector3i>>
      data_3d_;

  std::unordered_map<
      Eigen::Vector4i,
      PathNodePtr,
      matrix_hash<Eigen::Vector4i>>
      data_4d_;

public:
  NodeHashTable() {}

  ~NodeHashTable() {}

  void insert(Eigen::Vector3i idx, PathNodePtr node)
  {
    data_3d_.insert(std::make_pair(idx, node));
  }

  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
  {
    data_4d_.insert(
        std::make_pair(
            Eigen::Vector4i(
                idx(0),
                idx(1),
                idx(2),
                time_idx),
            node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);

    return iter == data_3d_.end()
               ? NULL
               : iter->second;
  }

  PathNodePtr find(Eigen::Vector3i idx, int time_idx)
  {
    auto iter = data_4d_.find(
        Eigen::Vector4i(
            idx(0),
            idx(1),
            idx(2),
            time_idx));

    return iter == data_4d_.end()
               ? NULL
               : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }
};


/**
 * @brief Fast-Planner Kinodynamic A*
 *
 * 当前正在从 Fast-Planner 的 EDTEnvironment
 * 迁移到 Elastic Tracker 的 mapping::OccGridMap。
 */
class KinodynamicAstar
{
private:
  /* =========================================================
   * 搜索主要数据结构
   * ========================================================= */

  std::vector<PathNodePtr> path_node_pool_;

  int use_node_num_;
  int iter_num_;

  NodeHashTable expanded_nodes_;

  std::priority_queue<
      PathNodePtr,
      std::vector<PathNodePtr>,
      NodeComparator>
      open_set_;

  std::vector<PathNodePtr> path_nodes_;


  /* =========================================================
   * 搜索状态记录
   * ========================================================= */

  Eigen::Vector3d start_vel_;
  Eigen::Vector3d end_vel_;
  Eigen::Vector3d start_acc_;

  // 状态转移矩阵
  Eigen::Matrix<double, 6, 6> phi_;


  /* =========================================================
   * 地图
   * ========================================================= */

  /**
   * 原 Fast-Planner：
   *
   * EDTEnvironment::Ptr edt_environment_;
   *
   * 现在替换为 Elastic Tracker 自己的地图。
   */
  std::shared_ptr<mapping::OccGridMap> map_ptr_;


  /* =========================================================
   * Shot trajectory
   * ========================================================= */

  bool is_shot_succ_ = false;

  Eigen::MatrixXd coef_shot_;

  double t_shot_;

  bool has_path_ = false;


  /* =========================================================
   * 搜索参数
   * ========================================================= */

  double max_tau_;
  double init_max_tau_;

  double max_vel_;
  double max_acc_;

  double w_time_;
  double horizon_;
  double lambda_heu_;

  int allocate_num_;
  int check_num_;

  double tie_breaker_;

  bool optimistic_;


  /* =========================================================
   * 地图 / 时间离散参数
   * ========================================================= */

  double resolution_;
  double inv_resolution_;

  double time_resolution_;
  double inv_time_resolution_;

  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_3d_;

  double time_origin_;


  /* =========================================================
   * 辅助函数
   * ========================================================= */

  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);

  int timeToIndex(double time);

  void retrievePath(PathNodePtr end_node);


  /* =========================================================
   * Shot trajectory
   * ========================================================= */

  std::vector<double> cubic(
      double a,
      double b,
      double c,
      double d);

  std::vector<double> quartic(
      double a,
      double b,
      double c,
      double d,
      double e);

  bool computeShotTraj(
      Eigen::VectorXd state1,
      Eigen::VectorXd state2,
      double time_to_goal);

  double estimateHeuristic(
      Eigen::VectorXd x1,
      Eigen::VectorXd x2,
      double &optimal_time);


  /* =========================================================
   * 状态传播
   * ========================================================= */

  void stateTransit(
      Eigen::Matrix<double, 6, 1> &state0,
      Eigen::Matrix<double, 6, 1> &state1,
      Eigen::Vector3d um,
      double tau);


public:
  KinodynamicAstar() {}

  ~KinodynamicAstar();


  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3,
    NEAR_END = 4
  };


  /* =========================================================
   * 主接口
   * ========================================================= */

  void setParam(ros::NodeHandle &nh);

  void init();

  void reset();


  /**
   * @brief Kinodynamic A* 搜索
   *
   * start_pt  起始位置
   * start_vel 起始速度
   * start_acc 起始加速度
   * end_pt    目标位置
   * end_vel   目标速度
   */
  int search(
      Eigen::Vector3d start_pt,
      Eigen::Vector3d start_vel,
      Eigen::Vector3d start_acc,
      Eigen::Vector3d end_pt,
      Eigen::Vector3d end_vel,
      bool init,
      bool dynamic = false,
      double time_start = -1.0);


  /**
   * @brief 设置 Elastic Tracker 的占据栅格地图
   *
   * 原版 Fast-Planner 这里接收 EDTEnvironment。
   */
  void setEnvironment(
      const std::shared_ptr<mapping::OccGridMap> &map);


  /**
   * @brief 将搜索结果按时间 delta_t 采样
   */
  std::vector<Eigen::Vector3d> getKinoTraj(
      double delta_t);


  void getSamples(
      double &ts,
      std::vector<Eigen::Vector3d> &point_set,
      std::vector<Eigen::Vector3d> &start_end_derivatives);


  std::vector<PathNodePtr> getVisitedNodes();


  typedef std::shared_ptr<KinodynamicAstar> Ptr;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner

#endif