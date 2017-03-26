// Copyright 2015 Caoyang Jiang

#ifndef MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_PROBABILITYMODEL_H_
#define MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_PROBABILITYMODEL_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <utility>
JCY_WINDOWS_ENABLE_ALL_WARNING

namespace jcy
{
//!< <character, codeword>
typedef std::map<char, uint64_t> FrequencyTable;
typedef std::map<char, double> ProbabilityTable;
typedef std::pair<char, double> ProbabilityPair;

class SymbolTable
{
 public:
  SymbolTable() : muiTotalCount(0)
  {
  }
  ~SymbolTable()
  {
  }

  void FillTable(const char *cstr, uint32_t size);
  void ShowFrequencyTable() const;
  void ShowProbabilityTable() const;
  size_t GetAlphabetaCount() const;

  const FrequencyTable &getFrequencyTable() const;
  const ProbabilityTable &getProbabilityTable() const;

 private:
  FrequencyTable mFrequencyTable;
  ProbabilityTable mProbabilityTable;
  size_t muiTotalCount;
};

class ProbabilityModel
{
 public:
  ProbabilityModel()
      : mbUpdateHistory(false), mbIsHistoryLoaded(false), mbUseHistory(false)
  {
  }
  ~ProbabilityModel()
  {
  }

  bool LoadHistory(std::string FileName);
  bool WriteHisory();

  void AnalyzeSymbolProbability(const std::string &cTextString);
  void AnalyzeSymbolProbability(std::fstream &FileName);
  void ShowSymbolProbability() const;

  bool IsUpdateHistory() const;
  bool IsHistoryLoaded() const;
  bool IsUseHistory() const;

  const SymbolTable &GetHistoryTable() const;
  const SymbolTable &GetCurrentTable() const;
  const ProbabilityTable &GetProbabilityTable() const;

 private:
  bool mbUpdateHistory;
  bool mbIsHistoryLoaded;
  bool mbUseHistory;

  SymbolTable mcHistoryTable;
  SymbolTable mcCurrentTable;
};
}  // namespace jcy

#endif  // MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_PROBABILITYMODEL_H_
