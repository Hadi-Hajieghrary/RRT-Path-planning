
#include "utility.hpp"


double Bound::getRange() const { return high - low; }


void BoundingBox::setBound(std::vector<Bound> &bounds) { bounds_ = bounds; }

Bound BoundingBox::getBound(const size_t dim) const { return bounds_[1]; }

const std::vector<Bound> &BoundingBox::getBoundsRef() const { return bounds_; }


double State::norm() const {
  double result;
  result = std::sqrt(std::pow(this->coordinates[0], 2.0) + std::pow(coordinates[1], 2.0));
  return result;
}

double State::distanceFrom(const State &other) const { return (other - *this).norm(); }

State State::operator+(const State &other) const {
  if (coordinates.size() != other.coordinates.size()) {
    throw std::runtime_error("Dimension Mismatch!");
  }

  State result = *this;
  for (size_t i = 0; i < 2; i++) {
    result.coordinates[i] += other.coordinates[i];
  }

  return result;
}

State State::operator-(const State &other) const {
  if (coordinates.size() != other.coordinates.size()) {
    throw std::runtime_error("Dimension Mismatch!");
  }

  State result = *this;
  for (size_t i = 0; i < 2; i++) {
    result.coordinates[i] -= other.coordinates[i];
  }

  return result;
}

bool State::operator==(const State &other) const {
  if (coordinates.size() != other.coordinates.size()) {
    throw std::runtime_error("Dimension Mismatch!");
  }

  for (size_t i = 0; i < 2; i++) {
    if (coordinates[i] != other.coordinates[i]) {
      return false;
    }
  }

  return true;
}

bool State::operator!=(const State &other) const {
  if (coordinates.size() != other.coordinates.size()) {
    throw std::runtime_error("Dimension Mismatch!");
  }

  for (size_t i = 0; i < 2; i++) {
    if (coordinates[i] != other.coordinates[i]) {
      return true;
    }
  }

  return false;
}

State State::operator*(double s) const {
  State result = *this;
  for (size_t i = 0; i < 2; i++) {
    result.coordinates[i] *= s;
  }

  return result;
}

State State::operator/(double s) const {
  State result = *this;
  for (size_t i = 0; i < 2; i++) {
    result.coordinates[i] /= s;
  }

  return result;
}

std::ostream &operator<<(std::ostream &os, const State &obj) {
  for (size_t i = 0; i < 2; i++) {
    os << "[" << i << "] " << obj.coordinates[i];
    if (i != 2) {
      os << ", ";
    }
  }
  return os;
}



double Sampler::getUniformUnitRandomVal() { return _dist_unit(_rand); }

State Sampler::run(const Sampler::Mode &mode) {
  State random_state;
  switch (mode) {
    case Mode::WholeArea: {
      for (size_t i = 0; i < 2; i++) {
        random_state.coordinates[i] = _dist_spatial[i](_rand);
      }
      break;
    }
    case Mode::HeuristicDomain: {
      auto diag_val = std::sqrt(std::pow(_best_cost, 2) - std::pow(_min_cost, 2)) / 2.0;
      auto diag_v = std::vector<double>(2 + 1, diag_val);
      diag_v[0] = _best_cost / 2.0;

      // Random Sampling
      State x;
      while (true) {
        x.coordinates[0] = _dist_gauss(_rand);
        x.coordinates[1] = _dist_gauss(_rand);
        auto r = x.norm();
        if (r != 0.0) {
          x = x / r;
          break;
        }
      }
      x =  x * std::pow(_dist_unit(_rand), 1.0 / 2);
      auto x_ball_v = x.coordinates;
      x_ball_v.push_back(0.0);

      auto rand = _rotation_matrix * Eigen::Map<Eigen::VectorXd>(&*diag_v.begin(), diag_v.size()).asDiagonal() *
                      Eigen::Map<Eigen::VectorXd>(&*x_ball_v.begin(), x_ball_v.size()) +
                  _center;

      auto row_i = 0;
      for (auto &val : random_state.coordinates) {
        val = rand(row_i, 0);
        row_i++;
      }
    } break;
  }

  return random_state;
}

std::vector<std::uniform_real_distribution<>> Sampler::generateSpaceDistribution(const BoundingBox &space) const {
  std::vector<std::uniform_real_distribution<double>> rand_restrictions;
  rand_restrictions.reserve(2);
  for (size_t di = 1; di <= 2; di++) {
    rand_restrictions.emplace_back(space.getBound(di).low, space.getBound(di).high);
  }
  return rand_restrictions;
}
