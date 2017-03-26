// Copyright 2015 Caoyang Jiang

#ifndef MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_CONTRACT_H_
#define MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_CONTRACT_H_

#include <iostream>

namespace jcy
{
#if defined(CONTRACT_PRECONDITION_ON) || defined(CONTRACT_ALL_ON)
#define Require(L, X) Check(#L, X, #X, __FILE__, __LINE__, "Client")
#else
#define Require(L, X) Noop()
#endif

#if defined(CONTRACT_POSTCONDITION_ON) || defined(CONTRACT_ALL_ON)
#define Ensure(L, X) Check(#L, X, #X, __FILE__, __LINE__, "Server")
#else
#define Ensure(L, X) Noop()
#endif

#if defined(CONTRACT_INVARIANT_ON) || defined(CONTRACT_ALL_ON)
#define Invariant(X) Check("invariant", X, #X, __FILE__, __LINE__, "invariant")
#else
#define Invariant(X) Noop()
#endif

class Contract
{
 public:
  Contract();
  ~Contract();

  static void Check(const char* label,
                    bool expr,
                    const char* expr_str,
                    const char* filename,
                    uint64_t line,
                    const char* src_str);

  static inline void Noop()
  {
  }
};
}  // namespace jcy
#endif  // MODULES_HUFFMAN_INCLUDE_JCY_HUFFMAN_CONTRACT_H_
