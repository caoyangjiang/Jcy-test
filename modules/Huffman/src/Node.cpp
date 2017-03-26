// Copyright 2015 Caoyang Jiang

#include "Jcy/Huffman/Node.h"

namespace jcy
{
template class GenericNode<char>;

template <typename T>
void GenericNode<T>::SetValue(const T tValue)
{
  mValue = tValue;
}

template <typename T>
T GenericNode<T>::GetValue() const
{
  return mValue;
}

template <typename T>
void UniDirNode<T>::SetNext(UniDirNode<T> *next)
{
  mpcNext = next;
}

template <typename T>
UniDirNode<T> *UniDirNode<T>::GetNext() const
{
  return mpcNext;
}

template <typename T>
void TreeNode<T>::SetRight(TreeNode<T> *node)
{
  mpcRight = node;
}

template <typename T>
void TreeNode<T>::SetLeft(TreeNode<T> *node)
{
  mpcLeft = node;
}

template <typename T>
TreeNode<T> *TreeNode<T>::GetRight() const
{
  return mpcRight;
}

template <typename T>
TreeNode<T> *TreeNode<T>::GetLeft() const
{
  return mpcLeft;
}
}  // namespace jcy
