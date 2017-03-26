// Copyright 2015 Caoyang Jiang

#ifndef MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_NODE_H_
#define MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_NODE_H_

namespace jcy
{
// Generic node class
template <typename T>
class GenericNode
{
 public:
  GenericNode()
  {
  }
  explicit GenericNode(T tValue) : mValue(tValue)
  {
  }

  virtual ~GenericNode()
  {
  }
  virtual void SetValue(const T tValue);
  virtual T GetValue() const;

 protected:
  T mValue;
};

// Implement uni-directional node using generic node
template <typename T>
class UniDirNode : public GenericNode<T>
{
 public:
  UniDirNode() : GenericNode<T>(), mpcNext(0)
  {
  }
  explicit UniDirNode(T cValue) : GenericNode<T>(cValue), mpcNext(0)
  {
  }
  ~UniDirNode() override
  {
  }

  void SetNext(UniDirNode<T> *next);
  UniDirNode<T> *GetNext() const;

 protected:
  UniDirNode<T> *mpcNext;
};

// Implement tree node using generic node
template <typename T>
class TreeNode : public GenericNode<T>
{
 public:
  TreeNode() : GenericNode<T>(), mpcLeft(0), mpcRight(0)
  {
  }
  explicit TreeNode(T value) : GenericNode<T>(value), mpcLeft(0), mpcRight(0)
  {
  }
  ~TreeNode() override
  {
  }

  void SetRight(TreeNode<T> *node);
  void SetLeft(TreeNode<T> *node);

  TreeNode<T> *GetRight() const;
  TreeNode<T> *GetLeft() const;

 protected:
  TreeNode<T> *mpcLeft;
  TreeNode<T> *mpcRight;
};
}  // namespace jcy

#endif  // MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_NODE_H_
