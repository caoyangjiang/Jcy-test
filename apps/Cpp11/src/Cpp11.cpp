// Copyright @ 2016 Caoyang Jiang

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <typeinfo>
#include <valarray>
#include <vector>

#define N 10000000

namespace WeakPtr
{
class Dummy
{
 public:
  void Hello()
  {
    std::cout << "Hello" << std::endl;
  }
};

int WeakPtrTest()
{
  std::shared_ptr<Dummy> ptr = std::make_shared<Dummy>();
  std::weak_ptr<Dummy> a     = ptr;

  if (a.expired())
  {
    std::cout << "Nothing done" << std::endl;
  }
  else
  {
    std::cout << "still alive" << std::endl;
  }

  std::shared_ptr<Dummy> ptr2 = a.lock();
  if (ptr2 != NULL)
  {
    ptr2->Hello();
  }
  else
  {
    std::cout << "Nothing done" << std::endl;
  }
  return 0;
}
}  // namespace WeakPtr

namespace Bind
{
class BindFunction
{
 public:
  struct Mul
  {
    int32_t a_ = 0;
    int32_t b_ = 0;
    int32_t Mul(bool& ec)
    {
      ec = true;
      return a_ * b_;
    }
  };

 public:
  void Run()
  {
    std::function<int32_t(bool&)> f0 =
        std::bind(&BindFunction::Get, this, std::placeholders::_1);
    RunGet(f0);

    auto f1 = std::bind(&BindFunction::Add, this, 3, 3, true);
    RunAdd(f1);
    auto f2 = std::bind(&BindFunction::Sub, this, 12, 3, std::placeholders::_1);
    RunSub(f2);
  }

  void RunGet(std::function<int32_t(bool&)> func)
  {
    bool ec = false;
    std::cout << "Ret: " << func(ec);
    std::cout << " EC: " << ec << std::endl;
  }

  void RunAdd(std::function<int32_t(void)> func)
  {
    std::cout << "Ret: " << func() << std::endl;
  }

  void RunSub(std::function<int32_t(bool&)> func)
  {
    bool ec = false;
    std::cout << "Ret: " << func(ec) << std::endl;
  }

 private:
  int32_t Get(bool& ec)
  {
    ec = true;
    return 123;
  }

  int32_t Add(int32_t a, int32_t b, bool ec)
  {
    std::cout << "Add ec: " << ec << std::endl;
    return a + b;
  }

  int32_t Sub(int32_t a, int32_t b, bool& ec)
  {
    if (a > b)
      ec = false;
    else
      ec = true;
    return a - b;
  }
};

void BindTest()
{
  BindFunction bf;
  bf.Run();
}

}  // namespace Bind

namespace HashTest
{
void HashLibraryTest()
{
  std::vector<std::string> vals;
  std::hash<std::string> strhash;

  vals.push_back("Caoyang");
  vals.push_back("Jiang");
  vals.push_back("Wei");
  vals.push_back("He");

  for (size_t i = 0; i < vals.size(); i++)
  {
    std::cout << strhash(vals[i]) << std::endl;
  }
}
}  // namespace HashTest

namespace RR
{
class RightReference
{
 public:
  RightReference()
  {
    std::cout << "Invoke default constructor" << std::endl;
  }

  RightReference(const RightReference& rr)
  {
    this->vec_ = rr.vec_;
    std::cout << "Invoke copy constructor" << std::endl;
  }

  RightReference(RightReference&& rr)
  {
    this->vec_ = rr.vec_;
    std::cout << "Invoke right reference constructor" << std::endl;
  }

  std::vector<int> vec_;
};

RightReference Create()
{
  RightReference tmp;
  tmp.vec_.push_back(1);
  return tmp;
}

void RightReferenceTest()
{
  RightReference first;
  RightReference second = first;
  RightReference third  = std::move(first);
  RightReference fourth = std::move(Create());
}
}  // namespace RR

namespace DeclType
{
auto Multiple(float a, int32_t b)
{
  // decltype derive correct type during compile time
  // typeid gets the type info
  decltype(a * b) c = a * b;
  return c;
}

template <typename T>
class TypeDetect
{
 public:
  static void IsInt32()
  {
    const std::type_info& ti32    = typeid(int32_t);
    const std::type_info& tui32   = typeid(uint32_t);
    const std::type_info& ti64    = typeid(int64_t);
    const std::type_info& tui64   = typeid(uint64_t);
    const std::type_info& tfloat  = typeid(float);
    const std::type_info& tdouble = typeid(double);
    const std::type_info& t2      = typeid(T);

    if (ti32 == t2)
    {
      std::cout << "Type is int32_t" << std::endl;
    }
    else if (tui32 == t2)
    {
      std::cout << "Type is uint32_t" << std::endl;
    }
    else if (tui64 == t2)
    {
      std::cout << "Type is int64_t" << std::endl;
    }
    else if (ti64 == t2)
    {
      std::cout << "Type is uint64_t" << std::endl;
    }
    else if (tfloat == t2)
    {
      std::cout << "Type is float" << std::endl;
    }
    else if (tdouble == t2)
    {
      std::cout << "Type is double" << std::endl;
    }
    else
    {
      std::cout << "Not detected" << std::endl;
    }
  }
};

void DeclTypeTest()
{
  auto result = Multiple(8, 8);
  TypeDetect<decltype(result)>::IsInt32();
}
}  // namespace DeclType

namespace VALARRAY
{
void ValArrayTest()
{
  // valarray is a "value array" where a operation can be applied on all
  // elements.
  //
  // This may take advantage of modern CPU instructions and acceleration
  // processing.
  std::valarray<float> angles = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};
  std::valarray<float> result = std::cos(angles);
  std::cout << "Angles: ";
  for (size_t i = 0; i < result.size(); i++)
  {
    std::cout << result[i] << " ";
  }

  std::cout << std::endl;
}
}  // namespace VALARRAY

namespace LAMBDA
{
void LambdaTest()
{
  std::default_random_engine generator;
  std::uniform_int_distribution<uint16_t> dist(0, 0xFF);
  std::chrono::high_resolution_clock::time_point time =
      std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::duration dtn = time.time_since_epoch();
  generator.seed(dtn.count());

  // A lambda function for generating random number
  auto mylambdafunc = [&dist, &generator ]() -> auto
  {
    return dist(generator);
  };

  std::vector<uint16_t> vals(10);
  std::generate(vals.begin(), vals.end(), mylambdafunc);  // batch create

  std::cout << "10 Random values: ";
  for (size_t i = 0; i < vals.size(); i++) std::cout << vals[i] << " ";
  std::cout << std::endl;
}
}  // namespace LAMBDA

int main()
{
  std::cout << "Hash Test" << std::endl;
  HashTest::HashLibraryTest();
  std::cout << "Right Reference Test" << std::endl;
  RR::RightReferenceTest();
  std::cout << "DeclType Test" << std::endl;
  DeclType::DeclTypeTest();
  std::cout << "ValArray Test" << std::endl;
  VALARRAY::ValArrayTest();
  std::cout << "Lambad Test" << std::endl;
  LAMBDA::LambdaTest();
  std::cout << "Bind Test" << std::endl;
  Bind::BindTest();
  std::cout << "WeakPtr Test" << std::endl;
  WeakPtr::WeakPtrTest();
}
