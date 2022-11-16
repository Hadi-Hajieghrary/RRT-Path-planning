#include "KDTree.hpp"


void KDTree::add(const NodePtr &node) {
  if (node->_parent != nullptr) node->_parent->_is_leaf = false;
  _leaves.push_back(node);

  if (std::log2(_leaves.size()) * (1 / REBALANCE_RATIO) <= _depth) {
    // rebuild balanced kd-tree
    recurciveClearOut(_root);
    _root = nullptr;
    _depth = 0;
    std::vector<int> indices(_leaves.size());
    std::iota(indices.begin(), indices.end(), 0);
    _root = recursiveBuild(indices, 0, (int)_leaves.size(), 0);
  } else {
    // insert the node to kd-tree
    recursiveInsert(_root, _leaves.size() - 1, 0);
  }
}

void KDTree::init() {
  recurciveClearOut(_root);
  _root = nullptr;
  _depth = 0;
  _leaves.clear();
}

int KDTree::getSize() { return _leaves.size(); }

KDTree::NodePtr KDTree::searchNN(const NodePtr &node) {
  NodePtr ret_node;
  auto min_dist = std::numeric_limits<double>::max();

  recurciveSerachNearestNeighbor(node, _root, ret_node, min_dist);
  return ret_node;
}


std::vector<KDTree::NodePtr> KDTree::searchLeafs() {
  std::vector<NodePtr> ret_nodes;
  for (const auto &v : _leaves) {
    if (v->_is_leaf) {
      ret_nodes.push_back(v);
    }
  }
  return ret_nodes;
}

void KDTree::recurciveClearOut(const KDNodePtr &node) {
  if (node == nullptr) return;
  if (node->_right_child) {
    recurciveClearOut(node->_right_child);
    node->_right_child = nullptr;
  }
  if (node->_left_child) {
    recurciveClearOut(node->_left_child);
    node->_left_child = nullptr;
  }
}

KDTree::KDNodePtr KDTree::recursiveBuild(std::vector<int> &indices, const int &offset, const int &npoints,
                                                   const int &depth) {
  if (npoints <= 0) {
    return nullptr;
  }
  _depth = std::max(_depth, depth);

  const int axis = depth % 2;
  const int mid = (npoints - 1) / 2;
  auto comp = [&](const int &lhs, const int &rhs) {
    return _leaves[lhs]->_state.coordinates[axis] < _leaves[rhs]->_state.coordinates[axis];
  };
  std::nth_element(indices.begin() + offset, indices.begin() + offset + mid, indices.begin() + offset + npoints, comp);

  auto node = std::make_shared<KDTreeNode>();
  node->_index = indices[offset + mid];
  node->_axis = axis;
  node->_right_child = recursiveBuild(indices, offset, mid, depth + 1);
  node->_left_child = recursiveBuild(indices, offset + mid + 1, npoints - mid - 1, depth + 1);
  return node;
}

KDTree::KDNodePtr KDTree::recursiveInsert(const KDNodePtr &root, const int &new_node_index,
                                                    const int &depth) {
  auto axis = depth % 2;
  if (root == nullptr) {
    auto node = std::make_shared<KDTreeNode>();
    node->_index = new_node_index;
    node->_axis = axis;

    _depth = std::max(_depth, depth);
    if (_depth == 0) {
      _root = node;
    }
    return node;
  } else {
    if (_leaves[new_node_index]->_state.coordinates[axis] < (_leaves[root->_index]->_state.coordinates[axis])) {
      root->_right_child = recursiveInsert(root->_right_child, new_node_index, depth + 1);
    } else {
      root->_left_child = recursiveInsert(root->_left_child, new_node_index, depth + 1);
    }
    return root;
  }
}

void KDTree::recurciveSerachNearestNeighbor(const NodePtr &query, const KDNodePtr node, NodePtr &initial_guess, double &min_dist) const {
  if (node == nullptr) {
    return;
  }

  const NodePtr &train = _leaves[node->_index];
  const double dist = query->_state.distanceFrom(train->_state);
  if (dist < min_dist) {
    min_dist = dist;
    initial_guess = _leaves[node->_index];
  }

  const int axis = node->_axis;
  const int dir = query->_state.coordinates[axis] < train->_state.coordinates[axis] ? 0 : 1;
  recurciveSerachNearestNeighbor(query, dir == 0 ? node->_right_child : node->_left_child, initial_guess, min_dist);
  const double diff = fabs(query->_state.coordinates[axis] - train->_state.coordinates[axis]);
  if (diff < min_dist) {
    recurciveSerachNearestNeighbor(query, dir == 0 ? node->_left_child : node->_right_child, initial_guess, min_dist);
  }
}