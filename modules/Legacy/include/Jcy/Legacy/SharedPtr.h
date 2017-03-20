// Copyright 2016 Caoyang Jiang

#ifndef MODULES_LEGACY_INCLUDE_JCY_LEGACY_SHAREDPTR_H_
#define MODULES_LEGACY_INCLUDE_JCY_LEGACY_SHAREDPTR_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>

namespace jcy
{
class Dummy
{
 public:
  Dummy()
  {
  }
  ~Dummy()
  {
  }
};

class SharedPtrTest
{
 public:
  SharedPtrTest();
  ~SharedPtrTest();

  boost::shared_ptr<Dummy> MakeSharedPtr() const;
};
}  // namespace jcy
#endif  // MODULES_LEGACY_INCLUDE_JCY_LEGACY_SHAREDPTR_H_
