// Copyright @ 2017 Caoyang Jiang

#ifndef MODULES_IESERVER_INCLUDE_JCY_IESERVER_CHATCLIENT_H_
#define MODULES_IESERVER_INCLUDE_JCY_IESERVER_CHATCLIENT_H_

#include <cstdlib>
#include <deque>
#include <iostream>
#include "Jcy/IEServer/ChatMessage.h"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "boost/thread.hpp"

namespace jcy
{
using boost::asio::ip::tcp;

typedef std::deque<chat_message> chat_message_queue;

class chat_client
{
 public:
  chat_client(boost::asio::io_service& io_service,
              tcp::resolver::iterator endpoint_iterator);

  void write(const chat_message& msg);

  void close();

  void handle_connect(const boost::system::error_code& error,
                      tcp::resolver::iterator endpoint_iterator);

  void handle_read_header(const boost::system::error_code& error);

  void handle_read_body(const boost::system::error_code& error);

  void do_write(chat_message msg);

  void handle_write(const boost::system::error_code& error);

  void do_close();

 private:
  boost::asio::io_service& io_service_;
  tcp::socket socket_;
  chat_message read_msg_;
  chat_message_queue write_msgs_;
};

}  // namespace jcy

#endif  // MODULES_IESERVER_INCLUDE_JCY_IESERVER_CHATCLIENT_H_
