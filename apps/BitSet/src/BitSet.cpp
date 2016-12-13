// Copyright @ 2016 Caoyang Jiang

#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <vector>

class Bitset
{
 public:
  Bitset();
  ~Bitset();

  bool operator[](size_t pos) const;

  // Initialize as a new bitset
  void Create();
  void Create(size_t preallocsize);

  // Create a new copy of an existing bitset
  void CopyTo(Bitset& bs) const;

  // Create a reference of existing bitset
  Bitset(const Bitset& bs);
  Bitset Subset(size_t beg, size_t end) const;
  Bitset& operator=(const Bitset& bs);

  // Modifiers
  void Pushback(bool bit);
  void Pushback(const Bitset& bs);
  void Insert(size_t pos, bool bit);
  void Insert(size_t pos, const Bitset& bs);
  void Delete(size_t pos);
  void Delete(size_t first, size_t last);
  void Mirror();
  void Clear();

  // Search
  int Find(int startpos, const Bitset& bs) const;
  int FindByte(int startpos, const Bitset& bs) const;

  // Query
  size_t Size() const;
  void ToByte(std::vector<uint8_t>& bytes) const;

 private:
  std::list<bool>::iterator GetItbyPos(size_t pos) const;
  std::list<bool>::iterator GetLast() const;
  std::list<bool>::iterator GetFirst() const;

 private:
  /* beg_ and begit_ points to the first element
   * end_ and endit_ points to the next avaliable spot (1 step passing the last)
   */
  typedef struct Info
  {
    size_t beg_;
    size_t end_;
    std::list<bool>::iterator begit_;
    std::list<bool>::iterator endit_;
  } Info;

  typedef struct Buffer
  {
    explicit Buffer(size_t size) : buf_(size), end_(buf_.end()), size_(size)
    {
    }

    Buffer(std::list<bool>::iterator first, std::list<bool>::iterator last)
        : buf_(first, last), end_(buf_.end()), size_(buf_.size())
    {
    }

    std::list<bool> buf_;
    std::list<bool>::iterator end_;
    size_t size_;
  } Buffer;

  Bitset* parent_ = nullptr;
  std::shared_ptr<Info> info_;
  std::shared_ptr<Buffer> bitbuf_;
};

std::list<bool>::iterator Bitset::GetItbyPos(size_t pos) const
{
  std::list<bool>::iterator it = info_->begit_;
  for (size_t ipos = 0; ipos < pos; ipos++) it++;
  return it;
}

std::list<bool>::iterator Bitset::GetFirst() const
{
  return info_->begit_;
}

std::list<bool>::iterator Bitset::GetLast() const
{
  return info_->endit_;
}

Bitset::Bitset()
{
}
Bitset::~Bitset()
{
}

void Bitset::Create(size_t preallocsize)
{
  bitbuf_       = std::shared_ptr<Buffer>(new Buffer(preallocsize));
  info_         = std::shared_ptr<Info>(new Info);
  info_->beg_   = 0;
  info_->end_   = 0;
  info_->begit_ = bitbuf_->buf_.begin();
  info_->endit_ = bitbuf_->buf_.begin();
}

void Bitset::Create()
{
  info_ = std::shared_ptr<Info>(new Info);
}

Bitset::Bitset(const Bitset& bs)
{
  info_   = bs.info_;
  bitbuf_ = bs.bitbuf_;
  parent_ = bs.parent_;
}

Bitset& Bitset::operator=(const Bitset& bs)
{
  info_   = bs.info_;
  bitbuf_ = bs.bitbuf_;
  parent_ = bs.parent_;
  return *this;
}

void Bitset::CopyTo(Bitset& bs) const
{
  /* subset will be cropped into a new full set*/
  bs.bitbuf_       = std::shared_ptr<Buffer>(new Buffer(GetFirst(), GetLast()));
  bs.info_         = std::shared_ptr<Info>(new Info);
  bs.info_->beg_   = 0;
  bs.info_->end_   = bs.bitbuf_->size_;
  bs.info_->begit_ = bs.bitbuf_->buf_.begin();
  bs.info_->endit_ = bs.bitbuf_->buf_.end();
  bs.parent_       = nullptr;
}

bool Bitset::operator[](size_t pos) const
{
  return *(GetItbyPos(pos));
}

void Bitset::Pushback(bool bit)
{
  if (info_->endit_ != bitbuf_->end_)
  {
    *((info_->endit_++)) = bit;
  }
  else
  {
    bitbuf_->buf_.insert((info_->endit_), bit);
  }

  info_->end_++;

  // Update parent
  Bitset* p = parent_;
  while (p != nullptr)
  {
    p->info_->end_++;
    p = p->parent_;
  }
}

void Bitset::Pushback(const Bitset& bs)
{
  Insert(info_->end_, bs);
}

Bitset Bitset::Subset(size_t beg, size_t end) const
{
  Bitset bs;

  bs.Create();
  bs.bitbuf_       = bitbuf_;
  bs.info_->beg_   = beg < info_->beg_ ? info_->beg_ : beg;
  bs.info_->end_   = end > info_->end_ ? info_->end_ : end;
  bs.info_->begit_ = info_->begit_;
  bs.info_->endit_ = info_->endit_;

  for (size_t ipos = info_->beg_; ipos < bs.info_->beg_; ipos++)
    (bs.info_->begit_)++;
  for (size_t ipos = info_->end_; ipos > bs.info_->end_; ipos--)
    (bs.info_->endit_)--;

  bs.parent_ = const_cast<Bitset*>(this);
  return bs;
}

void Bitset::Clear()
{
  bitbuf_->buf_.erase(info_->begit_, info_->endit_);
  if (parent_ != nullptr) parent_->info_->end_ -= info_->end_ - info_->beg_;
  info_->end_   = info_->beg_;
  info_->endit_ = info_->begit_;
}

size_t Bitset::Size() const
{
  return info_->end_ - info_->beg_;
}

void Bitset::ToByte(std::vector<uint8_t>& bytes) const
{
  std::list<bool>::iterator it = info_->begit_;

  for (size_t i = info_->beg_; i < info_->end_;)
  {
    uint8_t val = 0x00;
    for (size_t ib = 0; ib < 8; ib++, i++)
    {
      if (i < info_->end_)
      {
        val |= (*it << ib);
        it++;
      }
    }
    bytes.push_back(val);
  }
}

void Bitset::Mirror()
{
  std::list<bool>::iterator ff = info_->begit_;
  std::list<bool>::iterator bf = info_->endit_;
  bf--;
  bool val;

  while (bf != ff)
  {
    val = *bf;
    *bf = *ff;
    *ff = val;

    bf--;
    if (bf == ff) break;
    ff++;
  }
}

void Bitset::Insert(size_t pos, bool bit)
{
  bitbuf_->buf_.insert(GetItbyPos(pos), bit);
  info_->end_++;

  // Update parent
  Bitset* p = parent_;
  while (p != nullptr)
  {
    std::cout << p->info_->end_ << std::endl;
    p->info_->end_++;
    std::cout << p->info_->end_ << std::endl;
    p = p->parent_;
  }
}

void Bitset::Insert(size_t pos, const Bitset& bs)
{
  size_t bssize = bs.Size();
  bitbuf_->buf_.insert(GetItbyPos(pos), bs.GetFirst(), bs.GetLast());
  info_->end_ += bssize;

  // update parent
  Bitset* p = parent_;
  while (p != nullptr)
  {
    p->info_->end_ += bssize;
    p = p->parent_;
  }
}

void Bitset::Delete(size_t pos)
{
  info_->end_--;
  bitbuf_->buf_.erase(GetItbyPos(pos));

  // Update parent
  Bitset* p = parent_;
  while (p != nullptr)
  {
    p->info_->end_--;
    p = p->parent_;
  }
}

void Bitset::Delete(size_t first, size_t last)
{
  bitbuf_->buf_.erase(GetItbyPos(first), GetItbyPos(last));
  info_->end_ -= last - first;

  // update parent
  Bitset* p = parent_;
  while (p != nullptr)
  {
    p->info_->end_ -= last - first;
    p = p->parent_;
  }
}

int Bitset::Find(int startpos, const Bitset& bs) const
{
  std::list<bool>::iterator it   = GetItbyPos(startpos);
  std::list<bool>::iterator itbs = bs.info_->begit_;
  int pos                        = 0;
  int foundpos                   = -1;

  for (; it != info_->endit_; it++, pos++)
  {
    if ((*it == *itbs) && ((info_->end_ - info_->beg_ - pos) >=
                           (bs.info_->end_ - bs.info_->beg_)))
    {
      std::list<bool>::iterator itprev = it;
      foundpos                         = pos;
      for (; itbs != bs.info_->endit_; it++, itbs++)
      {
        if (*it != *itbs)
        {
          // Recover both iterators
          it       = itprev;
          itbs     = bs.info_->begit_;
          foundpos = -1;
          break;
        }
      }
    }

    if (foundpos != -1) break;
  }

  return foundpos;
}

int Bitset::FindByte(int startpos, const Bitset& bs) const
{
  if ((startpos % 8) != 0) return -1;

  int foundpos = -1;

  std::vector<uint8_t> srcbytes;
  std::vector<uint8_t> tarbytes;

  bs.ToByte(srcbytes);
  ToByte(tarbytes);
  size_t tarsize = tarbytes.size();
  size_t srcsize = srcbytes.size();

  for (size_t itar = 0; itar < tarsize; itar++)
  {
    if ((tarbytes[itar] == srcbytes[0]) && ((tarsize - itar) >= srcsize))
    {
      foundpos = itar * 8;
      for (size_t isrc = 0; isrc < srcsize; isrc++)
      {
        if (srcbytes[isrc] != tarbytes[itar + isrc])
        {
          foundpos = -1;
          break;
        }
      }
    }

    if (foundpos != -1) break;
  }

  return foundpos;
}

std::ostream& operator<<(std::ostream& os, Bitset& bs)
{
  for (size_t i = 0; i < bs.Size(); i++)
  {
    os << bs[i];
  }
  return os;
}

int main(int, char**)
{
  std::chrono::system_clock::time_point beg;
  std::chrono::system_clock::time_point end;
  std::chrono::duration<double, std::milli> milliseconds;

  Bitset bs[10];
  beg = std::chrono::high_resolution_clock::now();
  bs[0].Create(3000000);
  bs[1].Create(24);
  end          = std::chrono::high_resolution_clock::now();
  milliseconds = end - beg;
  std::cout << "allocate " << milliseconds.count() / 1000.0 << std::endl;

  beg = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < 1000000; i++) bs[0].Pushback(1);
  for (size_t i = 0; i < 1000000; i++) bs[0].Pushback(0);
  for (size_t i = 0; i < 8; i++) bs[1].Pushback(1);
  for (size_t i = 0; i < 16; i++) bs[1].Pushback(0);
  end           = std::chrono::high_resolution_clock::now();
  milliseconds  = end - beg;
  std::cout << "pushback " << milliseconds.count() / 1000.0 << std::endl;

  std::cout << bs[1] << std::endl;
  beg = std::chrono::high_resolution_clock::now();
  std::cout << "Found: " << bs[0].FindByte(0, bs[1]) << std::endl;
  end          = std::chrono::high_resolution_clock::now();
  milliseconds = end - beg;

  std::cout << "Search " << milliseconds.count() / 1000.0 << std::endl;

  Bitset bstest(bs[1]);

  bs[2] = bs[1];
  std::cout << bs[1] << std::endl;
  bstest.Mirror();
  std::cout << bs[2] << std::endl;
  bs[2] = bs[1].Subset(16, 24);
  std::cout << bs[2] << std::endl;

  // bs[2].Insert(8, bs[2]);
  // bs[2].Delete(0, 8);
  bs[2].Mirror();
  std::cout << bs[1].FindByte(0, bs[2]) << std::endl;
  std::cout << bs[2] << std::endl;
  std::cout << bs[1] << std::endl;
  bs[2].CopyTo(bs[3]);
  std::cout << bs[3] << std::endl;
  bs[3].Pushback(bs[3]);
  std::cout << bs[3] << std::endl;
  // Bitset bs2;

  // bs2 = bs.Subset(9, 11);
  // std::cout << bs.Size() << std::endl;

  // std::cout << bs2.Size() << " " << bs2 << std::endl;

  // Bitset bs3;

  // bs.CopyTo(bs3);
  // bs3.Mirror();
  // bs.Insert(9, bs2);
  // // bs.Insert(bs.Size() - 1, bs);
  // bs.CopyTo(bs3);
  // std::cout << "bs0 " << bs << std::endl;
  // bs3.Delete(9, 11);
  // std::cout << "bs3 " << bs3 << std::endl;

  // std::vector<uint8_t> src;
  // bs.ToByte(src);
  // for (size_t i = 0; i < src.size(); i++)
  //   std::cout << static_cast<uint32_t>(src[i]) << std::endl;

  // Bitset bs4;
  // bs4 = bs;

  // bs4.Resize(7);
  // std::cout << "bs4 " << bs << std::endl;
  return 0;
}
