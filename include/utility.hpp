#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include<utility>
#include <Eigen/Dense>

class Bound {
 public:
  double low;
  double high;

  explicit Bound() : low(0), high(0) {}

  explicit Bound(const double &_low, const double &_high) {
    if (_high < _low) {
      throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Bound is invalid");
    }
    high = _high;
    low = _low;
  }

  ~Bound() = default;

  double getRange() const;
};


class BoundingBox {
 public:

  explicit BoundingBox() { bounds_ = std::vector<Bound>(2, Bound(0, 0)); }

  ~BoundingBox() = default;


  void setBound(std::vector<Bound> &bounds);

  Bound getBound(const size_t dim) const;

  const std::vector<Bound> &getBoundsRef() const;

 private:
  std::vector<Bound> bounds_;
};


class State {
 public:
  std::vector<double> coordinates;

  explicit State() { coordinates = std::vector<double>(2); }

  State(const std::vector<double> _coordinates) : coordinates(_coordinates) {}

  ~State() = default;

  double norm() const;

  double dot(const State &other) const;

  double distanceFrom(const State &other) const;

  State operator+(const State &other) const;
  State operator-(const State &other) const;
  bool operator==(const State &other) const;
  bool operator!=(const State &other) const;
  State operator*(double s) const;
  State operator/(double s) const;

  friend std::ostream &operator<<(std::ostream &os, const State &obj);
};


class Sampler {
 public:
  enum class Mode { WholeArea, HeuristicDomain };

  explicit Sampler(const BoundingBox &space)
                : _rand(std::mt19937(std::random_device()())),
                  _dist_gauss(std::normal_distribution<>(0.0, 1.0)),
                  _dist_unit(std::uniform_real_distribution<>(0.0, 1.0)),
                  _dist_spatial(generateSpaceDistribution(space)),
                  _min_cost(0),
                  _best_cost(0),
                  _rotation_matrix(Eigen::MatrixXd::Identity(2, 2)),
                  _center(Eigen::VectorXd::Zero(2)) {}

  ~Sampler() = default; 

  double getUniformUnitRandomVal();

  State run(const Mode &mode = Mode::WholeArea);

 private:
  std::mt19937 _rand;
  std::normal_distribution<> _dist_gauss;
  std::uniform_real_distribution<> _dist_unit;
  std::vector<std::uniform_real_distribution<>> _dist_spatial;

  double _min_cost;
  double _best_cost;
  Eigen::MatrixXd _rotation_matrix;
  Eigen::VectorXd _center;

  std::vector<std::uniform_real_distribution<>> generateSpaceDistribution(const BoundingBox &space) const;
};


class Node {
 public:
  State _state;
  std::shared_ptr<Node> _parent;
  double _cost;
  double _cost_to_goal;
  bool _is_leaf;

  Node(const State &state, const std::shared_ptr<Node> parent, const double &cost = 0.0, const double &_cost_to_goal = std::numeric_limits<double>::max()): 
       _state(state), _parent(parent), _cost(cost), _cost_to_goal(_cost_to_goal), _is_leaf(true) {}
  ~Node() = default;
};



#endif