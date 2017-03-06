// Copyright @ 2017 Caoyang Jiang

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <list>
#include <set>
#include "Jcy/IEServer/ChatMessage.h"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "boost/enable_shared_from_this.hpp"
#include "boost/shared_ptr.hpp"

namespace jcy
{
using boost::asio::ip::tcp;

//----------------------------------------------------------------------

typedef std::deque<chat_message> chat_message_queue;

//----------------------------------------------------------------------

class chat_participant
{
 public:
  virtual ~chat_participant()
  {
  }
  virtual void deliver(const chat_message& msg) = 0;
};

typedef boost::shared_ptr<chat_participant> chat_participant_ptr;

//----------------------------------------------------------------------

class chat_room
{
 public:
  void join(chat_participant_ptr participant);
  void leave(chat_participant_ptr participant);
  void deliver(const chat_message& msg);

 private:
  std::set<chat_participant_ptr> participants_;
  enum
  {
    max_recent_msgs = 100
  };
  chat_message_queue recent_msgs_;
};

//----------------------------------------------------------------------

class chat_session : public chat_participant,
                     public boost::enable_shared_from_this<chat_session>
{
 public:
  chat_session(boost::asio::io_service& io_service, chat_room& room);
  tcp::socket& socket();
  void start();
  void deliver(const chat_message& msg) override;
  void handle_read_header(const boost::system::error_code& error);
  void handle_read_body(const boost::system::error_code& error);
  void handle_write(const boost::system::error_code& error);

 private:
  tcp::socket socket_;
  chat_room& room_;
  chat_message read_msg_;
  chat_message_queue write_msgs_;
};

typedef boost::shared_ptr<chat_session> chat_session_ptr;

//----------------------------------------------------------------------

class chat_server
{
 public:
  chat_server(boost::asio::io_service& io_service,
              const tcp::endpoint& endpoint);

  void handle_accept(chat_session_ptr session,
                     const boost::system::error_code& error);

 private:
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
  chat_room room_;
};

typedef boost::shared_ptr<chat_server> chat_server_ptr;
typedef std::list<chat_server_ptr> chat_server_list;

}  // namespace jcy
