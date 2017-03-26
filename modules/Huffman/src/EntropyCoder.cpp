// Copyright 2015 Caoyang Jiang

#include "Jcy/Huffman/EntropyCoder.h"

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <string>
JCY_WINDOWS_ENABLE_ALL_WARNING

namespace jcy
{
/*
void EntropyCoder::SetProbabilityTable (const ProbabilityTable&
rcProbabilityTable) {
        m_ProbabilityTable = pProbabilityTable;
}

void EntropyCoder::ShowCodewordTable() {
        for (CodewordTable::iterator it = m_CodewordTable.begin();
                 it != m_CodewordTable.end(); it++) {
                printf("%c %s\n",it->first,it->second.c_str());
        }
}*/

const CodewordTable &EntropyCoder::GetCodewordTable() const
{
  return mcCodewordTable;
}

/**
 * Branch left  appends '0' to the code word
 * Branch right appends '1' to the code word
 * Upon return previous appended binary will be
 * removed.
 */
void HuffmanCoder::AssignCodeword(HuffmanNode *pcNode)
{
  static std::string cCodeword;

  // std::cout << cCodeword << std::endl;
  if (pcNode->GetLeft() != 0)
  {
    cCodeword.push_back('0');
    AssignCodeword(pcNode->GetLeft());
    cCodeword.pop_back();
  }

  if (pcNode->GetRight() != 0)
  {
    cCodeword.push_back('1');
    AssignCodeword(pcNode->GetRight());
    cCodeword.pop_back();
  }

  mcCodewordTable[pcNode->GetValue().first] = cCodeword;
}

CodewordTable HuffmanCoder::GenerateCodeword(
    const ProbabilityTable &cProbabilityTable)
{
  /**
   * Step 1. (Push nodes into priority queue)
   * Symbols are sorted lowest probability to highest
   * as they are pushed onto processing queue. This
   * uses the property of the priority queue.
   */
  for (ProbabilityTable::const_iterator it = cProbabilityTable.begin();
       it != cProbabilityTable.end();
       it++)
  {
    mcProcessqueue.push(HuffmanNode(*it));
  }

  /**
   * Step 2. (Construct Huffman tree)
   * Two lowest probability symbols (Two front node) are removed.
   * They become the two children of a newly created node. Internal
   * node has probabilty of the sum of two children but has
   * no symbol. This node is pushed onto the process queue.
   * This process ends when only one node (Huffman root) lefts in the queue
   */
  while (mcProcessqueue.size() >
         1)  //!< End when only one node left in the queue
  {
    HuffmanNode *inNode = new HuffmanNode;
    inNode->SetLeft(new HuffmanNode(mcProcessqueue.top()));
    mcProcessqueue.pop();
    inNode->SetRight(new HuffmanNode(mcProcessqueue.top()));
    mcProcessqueue.pop();

    //!< Sum the two child node probability
    inNode->SetValue(
        ProbabilityPair(255,
                        inNode->GetLeft()->GetValue().second +
                            inNode->GetRight()->GetValue().second));
    mcProcessqueue.push(*inNode);
  }

  //!< Step 3. (Assign codeword according to Huffman tree)
  HuffmanNode *HuffmanTreeRoot = new HuffmanNode(mcProcessqueue.top());
  AssignCodeword(HuffmanTreeRoot);

  //!< Use binary tree class to delete HuffmanTree
  SimpleBinaryTree<ProbabilityPair> cTmpTree(HuffmanTreeRoot);
  cTmpTree.DestroyBinaryTree();

  //!< Return constructed codeword table
  return mcCodewordTable;
}
}  // namespace jcy
