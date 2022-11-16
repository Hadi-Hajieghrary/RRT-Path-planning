
#ifndef __KDTREE_HPP__
#define __KDTREE_HPP__


#include <limits>
#include <memory>

#include <utility.hpp>




class KDTree{
  using NodePtr = std::shared_ptr<Node>;

 public:
  explicit KDTree() :_root(nullptr), _depth(0) {}
  ~KDTree() = default;
  void add(const NodePtr &node);
  void init();
  int getSize();
  NodePtr searchNN(const NodePtr &node);
  std::vector<NodePtr> searchLeafs();

 private:
  const double REBALANCE_RATIO = 0.1;

  struct KDTreeNode {
    int _index;
    int _axis;
    std::shared_ptr<KDTreeNode> _right_child;
    std::shared_ptr<KDTreeNode> _left_child;
    KDTreeNode() : _index(-1), _axis(-1), _right_child(nullptr), _left_child(nullptr) {}
  };

  using KDNodePtr = std::shared_ptr<KDTreeNode>;

  KDNodePtr _root;
  std::vector<NodePtr> _leaves;
  int _depth;

  void recurciveClearOut(const KDNodePtr &node);

  KDNodePtr recursiveBuild(std::vector<int> &indices, const int &offset, const int &npoints, const int &depth);

  KDNodePtr recursiveInsert(const KDNodePtr &root, const int &new_node_index, const int &depth);

  void recurciveSerachNearestNeighbor(const NodePtr &query, const KDNodePtr node, NodePtr &initial_guess, double &min_dist) const;

};




#endif
