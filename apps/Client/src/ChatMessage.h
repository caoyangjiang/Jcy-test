// Copyright @ 2016 Caoyang Jiang

#ifndef APPS_CLIENT_SRC_CHATMESSAGE_H_
#define APPS_CLIENT_SRC_CHATMESSAGE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>

class chat_message
{
 public:
  enum
  {
    kHeader_length = 4
  };
  enum
  {
    kMax_body_length = 512
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

  std::size_t length() const
  {
    return kHeader_length + body_length_;
  }

  const char* body() const
  {
    return data_ + kHeader_length;
  }

  char* body()
  {
    return data_ + kHeader_length;
  }

  std::size_t body_length() const
  {
    return body_length_;
  }

  void body_length(std::size_t new_length)
  {
    body_length_                                      = new_length;
    if (body_length_ > kMax_body_length) body_length_ = kMax_body_length;
  }

  bool decode_header()
  {
    char header[kHeader_length + 1] = "";
    std::strncat(header, data_, kHeader_length);
    body_length_ = std::atoi(header);
    if (body_length_ > kMax_body_length)
    {
      body_length_ = 0;
      return false;
    }
    return true;
  }

  void encode_header()
  {
    char header[kHeader_length + 1] = "";
    std::snprintf(
        header, sizeof(header), "%4d", static_cast<int>(body_length_));
    std::memcpy(data_, header, kHeader_length);
  }

 private:
  char data_[kHeader_length + kMax_body_length];
  std::size_t body_length_;
};

#endif  // APPS_CLIENT_SRC_CHATMESSAGE_H_
