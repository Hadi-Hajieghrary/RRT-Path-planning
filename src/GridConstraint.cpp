#include "GridConstraint.hpp"


State GridConstraint::getGridIndex(const State &state) const {
  State index;
  for (size_t i = 0; i < 2; i++) {
    auto bound = space.getBound(i + 1);

    // return -1 if the state is out of range
    if (state.coordinates[i] < bound.low || bound.high < state.coordinates[i]) {
      return index;
    } else {
      index.coordinates[i] = (state.coordinates[i] - bound.low) * _dimentions[i] / bound.getRange();
    }
  }

  return index;
}

// Bresenham's line algorithm
std::vector<std::vector<int>> GridConstraint::calcLineIndices(State src_idx, State dst_idx) const {
  const auto dst_idx_orig = dst_idx;
  std::vector<std::vector<int>> line_indices;
  bool swap_dim = (std::fabs(dst_idx.coordinates[1] - src_idx.coordinates[1]) > std::fabs(dst_idx.coordinates[0] - src_idx.coordinates[0]));
  if (swap_dim) {
    std::swap(src_idx.coordinates[0], src_idx.coordinates[1]);
    std::swap(dst_idx.coordinates[0], dst_idx.coordinates[1]);
  }
  const auto idx_diff = dst_idx - src_idx;

  std::vector<double> delta{fabs(idx_diff.coordinates[0]), fabs(idx_diff.coordinates[1])};
  std::vector<int> step{(src_idx.coordinates[0] > dst_idx.coordinates[0]) ? -1 : 1, (src_idx.coordinates[1] > dst_idx.coordinates[1]) ? -1 : 1};

  double drift{delta[0] / 2.0};
  double v{src_idx.coordinates[1]};
  for (double x = src_idx.coordinates[0]; std::fabs(x - dst_idx.coordinates[0]) > 1.0; x += step[0]) {
    int cx = x;
    int cv = v;
    if (swap_dim) {
      std::swap(cv, cx);
    }
    line_indices.push_back(std::vector<int>{cx,cv});

    drift -= delta[1];
    if (drift < 0) {
      v += step[1];
      drift += delta[0];
    }
  }

  return line_indices;
}

bool GridConstraint::checkCollision(const State &src, const State &dst) const {
  const auto src_idx = getGridIndex(src);
  const auto dst_idx = getGridIndex(dst);
  for (size_t i = 0; i < 2; i++) {
    if (src_idx.coordinates[i] == -1 || dst_idx.coordinates[i] == -1) {
      return false;
    }
  }
  const auto line_indices = calcLineIndices(src_idx, dst_idx);
  for (const auto index : line_indices) {
    int constraint_array_index = 0;
    for (size_t i = 0; i < index.size(); i++) {
      if (index[i] < 0 || _dimentions[i] < index[i]) {
        return 0;
      }
      int product = 1;
      for (size_t j = 0; j < i; j++) {
        product *= _dimentions[j];
      }
      constraint_array_index += index[i] * product;
    }
    if (_constraint[constraint_array_index] == 0) {
      return false;
    }
  }
  return true;
}

bool GridConstraint::checkConstraintType(const State &state) const {
  // get index on single dimension array which correspond with state
  int index = 0;
  for (size_t i = 0; i < 2; i++) {
    auto bound = space.getBound(i + 1);

    // return NOENTRY Type if the state is out of range
    if (state.coordinates[i] < bound.low || bound.high < state.coordinates[i]) {
      return 0;
    }

    int product = 1;
    for (size_t j = 0; j < i; j++) {
      product *= _dimentions[j];
    }
    index += std::floor((state.coordinates[i] - bound.low) * _dimentions[i] / bound.getRange()) * product;
  }

  return _constraint[index];
}
