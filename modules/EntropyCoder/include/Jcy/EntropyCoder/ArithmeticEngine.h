// Copyright 2016 Caoyang Jiang

#ifndef MODULES_ENTROPYCODER_INCLUDE_JCY_ENTROPYCODER_ARITHMETICENGINE_H_
#define MODULES_ENTROPYCODER_INCLUDE_JCY_ENTROPYCODER_ARITHMETICENGINE_H_

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <map>
#include <memory>
#include <vector>
JCY_WINDOWS_ENABLE_ALL_WARNING

#include "Jcy/MiscTools/BitStream.h"

namespace jcy
{
template <typename T>
class ArithmeticEngine
{
 public:
  enum MODE
  {
    ENCODE,
    DECODE
  };

  enum PROBABILITYMODEL
  {
    LOWLOW = 0,
    LOWMID,
    LOWHIGH,
    MIDLOW,
    MIDMID,
    MIDHIGH,
    HIGHLOW,
    HIGHMID,
    HIGHHIGH
  };

 public:
  JCY_ENTROPYCODER_DLL explicit ArithmeticEngine(enum MODE mode);
  JCY_ENTROPYCODER_DLL ~ArithmeticEngine();

  /**
   * @brief      Loads a probability model externally.
   *
   * @param[in]  frequency  The frequency
   */
  JCY_ENTROPYCODER_DLL void LoadProbabilityModel(
      const std::vector<uint64_t>& frequency);

  /**
   * @brief      Loads a probability model internally
   *
   * @param[in]  emodel  The emodel
   */
  JCY_ENTROPYCODER_DLL void LoadProbabilityModel(enum PROBABILITYMODEL emodel,
                                                 uint64_t samplesize);

  /**
   * @brief      Encode input symbols.
   *
   * @param[in]  symbols      The symbols
   * @param[in]  totalsymbol  The totalsymbol
   */
  JCY_ENTROPYCODER_DLL void Encode(const T* symbols, size_t totalsymbol);

  /**
   * @brief      Support 400M number of symbols in the bitstream.
   *
   * @param[in]  bits         The bits
   * @param[in]  totalbits    The totalbits
   * @param[in]  totalsymbol  The totalsymbol
   *
   * @return     True if decoding successful, false otherwise.
   */
  JCY_ENTROPYCODER_DLL void Decode(const uint8_t* bits,
                                   size_t totalbits,
                                   size_t totalsymbol);

  /**
   * @brief      Clean internal coded bit buffer.
   */
  JCY_ENTROPYCODER_DLL void ResetBitBuffer();

  /**
   * @brief      Clean internal decoded symbol buffer.
   */
  JCY_ENTROPYCODER_DLL void ResetSymBuffer();

  /**
   * @brief      Restart arithmetic engine to initial state (not probability
   *             model if adaptive is selected.)
   */
  JCY_ENTROPYCODER_DLL void ResetEngineState();

  /**
   * @brief      Reset probability model to initial state.
   */
  JCY_ENTROPYCODER_DLL void ResetProbabilityModel();

  /**
   * @brief      Gets the coded bits.
   *
   * @return     The coded bits.
   */
  JCY_ENTROPYCODER_DLL const char* GetCodedBits() const;

  /**
   * @brief      Gets the coded bits count.
   *
   * @return     The coded bits count.
   */
  JCY_ENTROPYCODER_DLL size_t GetCodedBitsCount() const;

  /**
   * @brief      Gets the decoded symbols.
   *
   * @return     The decoded symbols.
   */
  JCY_ENTROPYCODER_DLL const T* GetDecodedSymbols() const;

  /**
   * @brief      Gets the number of  symbols.
   *
   * @return     Number of decoded symbols.
   */
  JCY_ENTROPYCODER_DLL size_t GetDecodedSymbolCount() const;

 private:
  void EncodeASymbol(T symbol);
  void EncodeFlush();
  T DecodeASymbol();

 private:
  const uint64_t kCodeValueBits_ = 32;
  const uint64_t kTopValue_      = 0xFFFFFFFF;  // (1 << 32) - 1
  const uint64_t kQtrValue_      = 0x40000000;  // (top / 4) + 1
  const uint64_t kHalfValue_     = 0x80000000;  // (2 * Qtr)
  const uint64_t k3QtrValue_     = 0xC0000000;  // (3 * Qtr)

  std::map<enum PROBABILITYMODEL, std::vector<int>> models_;
  std::vector<uint64_t> runtimemodel_;
  std::vector<uint64_t> backupmodel_;
  std::vector<T> decodedsymbols_;
  std::unique_ptr<BitStream> bs_;
  enum MODE mode_;

  uint64_t low_          = 0;
  uint64_t high_         = kTopValue_;
  uint64_t oppositebits_ = 0;
  uint64_t value_        = 0;
  uint64_t range_        = 0;
  uint64_t nextbit_      = 0;
  uint8_t bytebuffer_    = 0x00;
  uint8_t bytepos_       = 0x00;
};

}  // namespace jcy
#endif  // MODULES_ENTROPYCODER_INCLUDE_JCY_ENTROPYCODER_ARITHMETICENGINE_H_
