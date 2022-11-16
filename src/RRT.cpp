#include "RRT.hpp"




void RRT::setExpandDist(double goal_tolerance) noexcept { _max_reach_distance = goal_tolerance; }

bool RRT::solve(const State &start, const State &goal) {
  _node_list->init();
  _node_list->add(std::make_shared<Node>(start, nullptr));

  // Sampling
  int sampling_cnt = 0;
  std::shared_ptr<Node> end_node;
  while (true) {
    auto rand_node = std::make_shared<Node>(goal, nullptr);
    if (_random_sampler->getUniformUnitRandomVal() > 0.05) {
      rand_node->_state = _random_sampler->run(Sampler::Mode::WholeArea);
      if (_constraint->checkConstraintType(rand_node->_state) == 0) {
        continue;
      }
    }
    // The nearest node of tree to the sampled point
    auto nearest_node = _node_list->searchNN(rand_node);

    // generate new node
    auto new_node = pickNewNode(nearest_node, rand_node, _max_reach_distance);

    if (_constraint->checkCollision(nearest_node->_state, new_node->_state)) {
      _node_list->add(new_node);

      // terminate if distance between new node and goal state is less than 'goal_tolerance'
      if (new_node->_state.distanceFrom(goal) <= _max_reach_distance) {
        end_node = std::make_shared<Node>(goal, new_node);
        _node_list->add(end_node);
        break;
      }
    }

    sampling_cnt++;
    if (_max_sampling_num == sampling_cnt) {
      return false;
    }
  }

  result_.clear();
  auto cost = 0.0;
  while (true) {
    result_.insert(result_.begin(), end_node->_state);
    if (end_node->_parent == nullptr) {
      cost += end_node->_state.distanceFrom(start);
      break;
    } else {
      cost += end_node->_state.distanceFrom(end_node->_parent->_state);
    }

    end_node = end_node->_parent;
  }

  _result_cost = cost;
  return true;
}


void RRT::setUp(const std::shared_ptr<GridConstraint> &constraint) {
  _constraint = constraint;
  _random_sampler = std::make_unique<Sampler>(constraint->space);
}

void RRT::setSearchCostMaxLimit(const double &terminate_search_cost) {
  _cost_limit = terminate_search_cost;
}

const std::vector<State> &RRT::getResult() const { return result_; }

double RRT::getResultCost() const { return _result_cost; }

std::shared_ptr<KDTree> RRT::getNodeList() const { return _node_list; }

std::shared_ptr<Node> RRT::pickNewNode(const std::shared_ptr<Node> &src_node,
                                                     const std::shared_ptr<Node> &dst_node,
                                                     const double &goal_tolerance) const {
  auto steered_node = std::make_shared<Node>(src_node->_state, src_node, src_node->_cost);
  auto dist_src_to_dst = src_node->_state.distanceFrom(dst_node->_state);
  if (dist_src_to_dst < goal_tolerance) {
    steered_node->_cost += dist_src_to_dst;
    steered_node->_state = dst_node->_state;
  } else {
    steered_node->_cost += goal_tolerance;
    steered_node->_state = src_node->_state + ((dst_node->_state - src_node->_state) / dist_src_to_dst) * goal_tolerance;
  }
  return steered_node;
}


