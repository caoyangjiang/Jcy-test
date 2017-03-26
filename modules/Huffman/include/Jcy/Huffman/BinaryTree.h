// Copyright 2015 Caoyang Jiang

#ifndef MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_BINARYTREE_H_
#define MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_BINARYTREE_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <stdint.h>
#include <vector>
JCY_WINDOWS_ENABLE_ALL_WARNING

#include "Jcy/Huffman/Node.h"

namespace jcy
{
template <class T>
class GenericBinaryTree
{
 public:
  virtual void Traverse() = 0;
  virtual void ConstructBinaryTree(const T *array, uint32_t size) = 0;
  void DestroyBinaryTree();
  void SetRoot(TreeNode<T> *node);
  TreeNode<T> *GetRoot() const;

 protected:
  GenericBinaryTree() : mpcRoot(0)
  {
  }
  explicit GenericBinaryTree(TreeNode<T> *pcNode) : mpcRoot(pcNode)
  {
  }
  virtual ~GenericBinaryTree()
  {
  }

 private:
  void RecursiveDeletion(TreeNode<T> *node);

 protected:
  TreeNode<T> *mpcRoot;
};

template <class T>
class SimpleBinaryTree : public GenericBinaryTree<T>
{
 public:
  SimpleBinaryTree()
  {
  }
  explicit SimpleBinaryTree(TreeNode<T> *node) : GenericBinaryTree<T>(node)
  {
  }
  ~SimpleBinaryTree() override
  {
  }

  void Traverse();
  void ConstructBinaryTree(const T *array, uint32_t size);
};
}  // namespace jcy

#endif  // MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_BINARYTREE_H_
