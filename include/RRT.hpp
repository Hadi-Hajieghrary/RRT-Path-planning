#ifndef __RRT_HPP__
#define __RRT_HPP__


#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <random>
#include <vector>


#include <KDTree.hpp>
#include <GridConstraint.hpp>
#include <utility.hpp>


class RRT{
 public:
  RRT(const int &max_sampling_num,
          const double &goal_tolerance)
      : _cost_limit{0},
        _constraint{std::make_shared<GridConstraint>(BoundingBox())},
        _node_list{std::make_shared<KDTree>()}, _max_sampling_num{max_sampling_num},
        _max_reach_distance{goal_tolerance}
  {}

  ~RRT() = default;


  void setExpandDist(double goal_tolerance) noexcept;

  bool solve(const State &start, const State &goal);

  void setUp(const std::shared_ptr<GridConstraint> &constraint);

  void setSearchCostMaxLimit(const double &terminate_search_cost);

  const std::vector<State> &getResult() const;

  double getResultCost() const;

  std::shared_ptr<KDTree> getNodeList() const;

 private:

  std::vector<State> result_;
  double _result_cost;
  double _cost_limit;
  std::shared_ptr<GridConstraint> _constraint;
  std::shared_ptr<KDTree> _node_list;
  std::unique_ptr<Sampler> _random_sampler;

  int _max_sampling_num;
  double _max_reach_distance;

  std::shared_ptr<Node> pickNewNode(const std::shared_ptr<Node> &src_node, const std::shared_ptr<Node> &dst_node,
                                          const double &goal_tolerance) const;
};








#endif
