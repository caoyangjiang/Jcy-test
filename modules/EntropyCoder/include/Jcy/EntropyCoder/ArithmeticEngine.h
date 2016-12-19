// Copyright 2016 Caoyang Jiang

#ifndef MODULES_ENTROPYCODER_INCLUDE_JCY_ENTROPYCODER_ARITHMETICENGINE_H_
#define MODULES_ENTROPYCODER_INCLUDE_JCY_ENTROPYCODER_ARITHMETICENGINE_H_

HVR_WINDOWS_DISABLE_ALL_WARNING
#include <map>
#include <memory>
#include <vector>
HVR_WINDOWS_ENABLE_ALL_WARNING

#include "Jcy/MiscTools/BitStream.h"

namespace Jcy
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
  HVR_WINDOWS_DLL_API explicit ArithmeticEngine(enum MODE mode);
  HVR_WINDOWS_DLL_API ~ArithmeticEngine();

  /**
   * @brief      Loads a probability model externally.
   *
   * @param[in]  frequency  The frequency
   */
  HVR_WINDOWS_DLL_API void LoadProbabilityModel(const std::vector<uint64_t>& frequency);

  /**
   * @brief      Loads a probability model internally
   *
   * @param[in]  emodel  The emodel
   */
  HVR_WINDOWS_DLL_API void LoadProbabilityModel(enum PROBABILITYMODEL emodel, uint64_t samplesize);

  /**
   * @brief      Encode input symbols.
   *
   * @param[in]  symbols      The symbols
   * @param[in]  totalsymbol  The totalsymbol
   */
  HVR_WINDOWS_DLL_API void Encode(const T* symbols, size_t totalsymbol);

  /**
   * @brief      Support 400M number of symbols in the bitstream.
   *
   * @param[in]  bits         The bits
   * @param[in]  totalbits    The totalbits
   * @param[in]  totalsymbol  The totalsymbol
   *
   * @return     True if decoding successful, false otherwise.
   */
  HVR_WINDOWS_DLL_API void Decode(const uint8_t* bits, size_t totalbits, size_t totalsymbol);

  /**
   * @brief      Clean internal coded bit buffer.
   */
  HVR_WINDOWS_DLL_API void ResetBitBuffer();

  /**
   * @brief      Clean internal decoded symbol buffer.
   */
  HVR_WINDOWS_DLL_API void ResetSymBuffer();

  /**
   * @brief      Restart arithmetic engine to initial state (not probability
   *             model if adaptive is selected.)
   */
  HVR_WINDOWS_DLL_API void ResetEngineState();

  /**
   * @brief      Reset probability model to initial state.
   */
  HVR_WINDOWS_DLL_API void ResetProbabilityModel();

  /**
   * @brief      Gets the coded bits.
   *
   * @return     The coded bits.
   */
  HVR_WINDOWS_DLL_API const char* GetCodedBits() const;

  /**
   * @brief      Gets the coded bits count.
   *
   * @return     The coded bits count.
   */
  HVR_WINDOWS_DLL_API size_t GetCodedBitsCount() const;

  /**
   * @brief      Gets the decoded symbols.
   *
   * @return     The decoded symbols.
   */
  HVR_WINDOWS_DLL_API const T* GetDecodedSymbols() const;

 private:
  void EncodeASymbol(T symbol);
  void EncodeFlush();
  T DecodeASymbol();

 private:
  const uint64_t kCodeValueBits_ = 32;
  const uint64_t kTopValue_ = 0xFFFFFFFF;  // (1 << 32) - 1
  const uint64_t kQtrValue_ = 0x40000000;  // (top / 4) + 1
  const uint64_t kHalfValue_ = 0x80000000;  // (2 * Qtr)
  const uint64_t k3QtrValue_ = 0xC0000000;  // (3 * Qtr)

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

}  // namespace Jcy
#endif  // MODULES_ENTROPYCODER_INCLUDE_JCY_ENTROPYCODER_ARITHMETICENGINE_H_
