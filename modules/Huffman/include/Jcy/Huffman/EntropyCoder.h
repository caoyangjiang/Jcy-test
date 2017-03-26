// Copyright 2015 Caoyang Jiang

#ifndef MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_ENTROPYCODER_H_
#define MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_ENTROPYCODER_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>
JCY_WINDOWS_ENABLE_ALL_WARNING

#include "Jcy/Huffman/BinaryTree.h"
#include "Jcy/Huffman/ProbabilityModel.h"

namespace jcy
{
typedef std::map<char, std::string> CodewordTable;
typedef std::pair<char, std::string> Codeword;

class EntropyCoder
{
 public:
  EntropyCoder()
  {
  }
  virtual ~EntropyCoder()
  {
  }
  virtual CodewordTable GenerateCodeword(
      const ProbabilityTable &cProbabilityTable) = 0;

  const CodewordTable &GetCodewordTable() const;

 protected:
  CodewordTable mcCodewordTable;
};

//!< This class is not filled yet
class ArithmeticCoder : public EntropyCoder
{
 public:
  ArithmeticCoder()
  {
  }
  ~ArithmeticCoder() override
  {
  }
  CodewordTable GenerateCodeword(
      const ProbabilityTable &cProbabilityTable) override;
};

//!< This class implememts Huffman Coder
class HuffmanCoder : public EntropyCoder
{
 public:
  typedef TreeNode<ProbabilityPair> HuffmanNode;

  HuffmanCoder()
  {
  }
  ~HuffmanCoder() override
  {
  }
  CodewordTable GenerateCodeword(
      const ProbabilityTable &cProbabilityTable) override;

 private:
  class Predicate
  {
   public:
    /**
     * The value of Huffman node is probability pair (first=symobol,
     * second=probability).
     * Node with lower probability has higher priority and is placed in the
     * front of the queue.
     */
    bool operator()(const HuffmanNode &nodeA, const HuffmanNode &nodeB)
    {
      if (nodeA.GetValue().second > nodeB.GetValue().second)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  };  //!< This class is created for priority queue.

  //!< This method assigns codword to the Huffman leaves
  void AssignCodeword(HuffmanNode *root);

 private:
  std::priority_queue<HuffmanNode, std::vector<HuffmanNode>, Predicate>
      mcProcessqueue;
};
}  // namespace jcy
#endif  // MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_ENTROPYCODER_H_
