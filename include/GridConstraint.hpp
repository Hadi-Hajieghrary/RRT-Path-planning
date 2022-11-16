
#ifndef __GRID_CONSTRAINT_HPP_
#define __GRID_CONSTRAINT_HPP_

#include <utility.hpp>
#include <cstdint>
#include <iostream>
#include <vector>


class GridConstraint {
 public:
  BoundingBox space;

  explicit GridConstraint(const BoundingBox &_space) : space(_space) {}


  GridConstraint(const BoundingBox &_space, const std::vector<bool> &constraint,
                               const std::vector<int> &each_dim_size)
                              : space(_space) ,_constraint{std::move(constraint)} ,_dimentions{each_dim_size}{}

  ~GridConstraint() = default;

  State getGridIndex(const State &state) const;

  std::vector<std::vector<int>> calcLineIndices(State src_idx, State dst_idx) const;

  bool checkCollision(const State &src, const State &dst) const;

  bool checkConstraintType(const State &state) const;

 private:
  std::vector<bool> _constraint;
  std::vector<int> _dimentions;
};



#endif
