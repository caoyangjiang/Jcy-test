// Copyright 2015 Caoyang Jiang

#include "Jcy/Huffman/Contract.h"

JCY_WINDOWS_DISABLE_ALL_WARNING
#include <cstdio>
#include <cstdlib>
JCY_WINDOWS_ENABLE_ALL_WARNING

namespace jcy
{
void Contract::Check(const char* label,
                     bool expr,
                     const char* expr_str,
                     const char* filename,
                     uint64_t line,
                     const char* src_str)
{
  if (!expr)
  {
    fprintf(stderr,
            "%s Broken contract @ %s:%ld [%s, %s]\a\n",
            src_str,
            filename,
            line,
            label,
            expr_str);
    exit(1);
  }
}
}  // namespace jcy
