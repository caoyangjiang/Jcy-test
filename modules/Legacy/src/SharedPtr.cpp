// Copyright @ 2016 Caoyang Jiang

#include "Jcy/Legacy/SharedPtr.h"
#include <boost/smart_ptr/shared_ptr.hpp>

namespace Jcy
{
SharedPtrTest::SharedPtrTest()
{
}

SharedPtrTest::~SharedPtrTest()
{
}

boost::shared_ptr<Dummy> SharedPtrTest::MakeSharedPtr() const
{
  return boost::shared_ptr<Dummy>(new Dummy);
  // nothing
}
}  // namespace Jcy
