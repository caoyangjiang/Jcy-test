// Copyright 2015 Caoyang Jiang

#include "Jcy/Huffman/BinaryTree.h"

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
JCY_WINDOWS_ENABLE_ALL_WARNING

namespace jcy
{
template class GenericBinaryTree<char>;

template <typename T>
void GenericBinaryTree<T>::SetRoot(TreeNode<T> *node)
{
  mpcRoot = node;
}

template <typename T>
TreeNode<T> *GenericBinaryTree<T>::GetRoot() const
{
  return mpcRoot;
}

template <typename T>
void GenericBinaryTree<T>::RecursiveDeletion(TreeNode<T> *pNode)
{
  if (pNode->GetLeft() != 0) RecursiveDeletion(pNode->GetLeft());

  if (pNode->GetRight() != 0) RecursiveDeletion(pNode->GetRight());

  delete pNode;
}

template <typename T>
void GenericBinaryTree<T>::DestroyBinaryTree()
{
  RecursiveDeletion(mpcRoot);
  mpcRoot = 0;
}

//!< This method constructs a simple binary tree using built-in type array
template <typename T>
void SimpleBinaryTree<T>::ConstructBinaryTree(const T *pArray,
                                              uint32_t arraysize)
{
  std::queue<TreeNode<T> *> ChildNodes;
  TreeNode<T> *pcNode;
  uint32_t count = 0;
  bool bleft     = true;

  //!< Guard
  if (arraysize > 0)
  {
    GenericBinaryTree<T>::mpcRoot = new TreeNode<T>(pArray[count++]);
    ChildNodes.push(GenericBinaryTree<T>::mpcRoot);

    //!< breadth-first traversing. Two children nodes are pushed into a queue
    while (count < arraysize)
    {
      pcNode = new TreeNode<T>(pArray[count]);
      if (!pcNode)
      {
        std::cout << "Bad malloc @ BinaryTree.tcc -> SimpleBinaryTree<T> -> "
                     "ConstructBinaryTree(...) "
                  << std::endl;
        exit(1);
      }

      bleft ? ChildNodes.front()->SetLeft(pcNode)
            : ChildNodes.front()->SetRight(pcNode);
      ChildNodes.push(pcNode);

      //!< alternate until two child node are created.
      if (bleft == false)
      {
        ChildNodes.pop();
      }

      bleft = !bleft;
      count++;
    }
  }
}

template <typename T>
void SimpleBinaryTree<T>::Traverse()
{
}

}  // namespace jcy
