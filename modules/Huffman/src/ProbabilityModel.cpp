// Copyright 2015 Caoyang Jiang

#include "Jcy/Huffman/ProbabilityModel.h"

namespace jcy
{
void SymbolTable::FillTable(const char *cstr, uint32_t size)
{
  uint32_t i = 0;
  char c;

  //!< Fill frequency table
  while (i < size)
  {
    c = cstr[i];
    muiTotalCount++;
    mFrequencyTable[c]++;
    i++;
  }

  //!< Fill probability table
  for (FrequencyTable::iterator beg = mFrequencyTable.begin();
       beg != mFrequencyTable.end();
       beg++)
  {
    mProbabilityTable[beg->first] =
        static_cast<double>(beg->second) / muiTotalCount;
  }
}

const FrequencyTable &SymbolTable::getFrequencyTable() const
{
  return mFrequencyTable;
}

const ProbabilityTable &SymbolTable::getProbabilityTable() const
{
  return mProbabilityTable;
}

void SymbolTable::ShowFrequencyTable() const
{
  for (FrequencyTable::const_iterator beg = mFrequencyTable.begin();
       beg != mFrequencyTable.end();
       beg++)
  {
    printf("%2c %lu\n", beg->first, beg->second);
  }
}

void SymbolTable::ShowProbabilityTable() const
{
  for (ProbabilityTable::const_iterator beg = mProbabilityTable.begin();
       beg != mProbabilityTable.end();
       beg++)
  {
    printf("%2c %.6f\n", beg->first, beg->second);
  }
}

size_t SymbolTable::GetAlphabetaCount() const
{
  if (mFrequencyTable.size() != mProbabilityTable.size())
    return 0;
  else
    return (size_t)mFrequencyTable.size();
}

bool ProbabilityModel::IsUpdateHistory() const
{
  return mbUpdateHistory;
}

bool ProbabilityModel::IsHistoryLoaded() const
{
  return mbIsHistoryLoaded;
}

bool ProbabilityModel::IsUseHistory() const
{
  return mbUseHistory;
}

const SymbolTable &ProbabilityModel::GetHistoryTable() const
{
  return mcHistoryTable;
}

const SymbolTable &ProbabilityModel::GetCurrentTable() const
{
  return mcCurrentTable;
}

const ProbabilityTable &ProbabilityModel::GetProbabilityTable() const
{
  return (mbUseHistory && mbIsHistoryLoaded)
             ? mcHistoryTable.getProbabilityTable()
             : mcCurrentTable.getProbabilityTable();
}

void ProbabilityModel::AnalyzeSymbolProbability(const std::string &InputString)
{
  mcCurrentTable.FillTable(InputString.c_str(), InputString.size());
}

void ProbabilityModel::AnalyzeSymbolProbability(std::fstream &TextFile)
{
  //!< Buffer text file in string
  std::string cTextString((std::istreambuf_iterator<char>(TextFile)),
                          std::istreambuf_iterator<char>());
  mcCurrentTable.FillTable(cTextString.c_str(), cTextString.size());
}

void ProbabilityModel::ShowSymbolProbability() const
{
  if (mbUseHistory && mbIsHistoryLoaded)
  {
    mcHistoryTable.ShowProbabilityTable();
  }
  else
  {
    mcCurrentTable.ShowProbabilityTable();
  }
}

bool ProbabilityModel::LoadHistory(std::string)
{
  return false;
}
}  // namespace jcy
