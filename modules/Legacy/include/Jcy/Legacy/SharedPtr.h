// Copyright 2016 Caoyang Jiang

#ifndef MODULES_BOOST_INCLUDE_JCY_BOOST_SHAREDPTR_H_
#define MODULES_BOOST_INCLUDE_JCY_BOOST_SHAREDPTR_H_

#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>

namespace Jcy
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
}  // namespace Jcy
#endif  // MODULES_BOOST_INCLUDE_JCY_BOOST_SHAREDPTR_H_
