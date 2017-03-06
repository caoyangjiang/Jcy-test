// Copyright @ 2017 Caoyang Jiang

#ifndef CHAT_MESSAGE_HPP
#define CHAT_MESSAGE_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace jcy
{
class chat_message
{
 public:
  enum
  {
    header_length = 4
  };
  enum
  {
    max_body_length = 512
  };

  chat_message() : body_length_(0)
  {
  }

  const char* data() const
  {
    return data_;
  }

  char* data()
  {
    return data_;
  }

  size_t length() const
  {
    return header_length + body_length_;
  }

  const char* body() const
  {
    return data_ + header_length;
  }

  char* body()
  {
    return data_ + header_length;
  }

  size_t body_length() const
  {
    return body_length_;
  }

  void body_length(size_t length)
  {
    body_length_                                     = length;
    if (body_length_ > max_body_length) body_length_ = max_body_length;
  }

  bool decode_header()
  {
    char header[header_length + 1] = "";
    std::strncat(header, data_, header_length);
    body_length_ = std::atoi(header);
    if (body_length_ > max_body_length)
    {
      body_length_ = 0;
      return false;
    }
    return true;
  }

  void encode_header()
  {
    char header[header_length + 1] = "";
    std::snprintf(
        header, header_length + 1, "%4d", static_cast<int>(body_length_));
    std::memcpy(data_, header, header_length);
  }

 private:
  char data_[header_length + max_body_length];
  size_t body_length_;
};

}  // namespace jcy
#endif  // CHAT_MESSAGE_HPP
