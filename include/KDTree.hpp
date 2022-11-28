
#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__


#include <limits>
#include <memory>

#include <utility.hpp>



  struct KDTreeNode {
    int _index;
    int _axis;
    std::shared_ptr<KDTreeNode> _right_child;
    std::shared_ptr<KDTreeNode> _left_child;
    KDTreeNode() : _index(-1), _axis(-1), _right_child(nullptr), _left_child(nullptr) {}
  };

class KDTree{

 public:
  explicit KDTree() :_root(nullptr), _depth(0) {}
  ~KDTree() = default;
  void add(const std::shared_ptr<Node> &node);
  void init();
  int getSize();
  std::shared_ptr<Node> searchNN(const std::shared_ptr<Node> &node);
  std::vector<std::shared_ptr<Node>> searchLeaves();

 private:
  const double REBALANCE_RATIO = 0.1;

  std::shared_ptr<KDTreeNode> _root;
  std::vector<std::shared_ptr<Node>> _leaves;
  int _depth;

  void recursiveClearOut(const std::shared_ptr<KDTreeNode> &node);

  std::shared_ptr<KDTreeNode> recursiveBuild(std::vector<int> &indices, const int &offset, const int &npoints, const int &depth);

  std::shared_ptr<KDTreeNode> recursiveInsert(const std::shared_ptr<KDTreeNode> &root, const int &new_node_index, const int &depth);

  void recursiveSearchNearestNeighbor(const std::shared_ptr<Node> &query, const std::shared_ptr<KDTreeNode> node, std::shared_ptr<Node> &initial_guess, double &min_dist) const;

};




#endif
