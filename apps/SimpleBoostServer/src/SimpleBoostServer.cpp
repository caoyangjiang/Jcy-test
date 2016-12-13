#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <boost/array.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

using boost::asio::ip::tcp;

class RenderEngine
{
 public:
  RenderEngine& operator=(const RenderEngine&) = delete;
  RenderEngine(const RenderEngine&)            = delete;

  explicit RenderEngine(tcp::socket socket) : socket_(std::move(socket))
  {
  }

  void Start()
  {
    data_ = std::unique_ptr<uint8_t[]>(new uint8_t[4 * 1024 * 1024]);
    size_t filesize;
    std::unique_ptr<uint8_t[]> frame;

    std::ifstream fs(
        "/home/hypevr/Desktop/Caoyang_Shared/AndroidTestMaterial/"
        "test1088.h264",
        std::ifstream::in | std::ifstream::binary);
    fs.seekg(0, fs.end);
    filesize = fs.tellg();
    fs.seekg(0, fs.beg);

    frame = std::unique_ptr<uint8_t[]>(new uint8_t[filesize]);
    fs.read(reinterpret_cast<char*>(frame.get()), filesize);
    fs.close();

    // TODO(cjiang): Initialize graphic rendering
    // TODO(cjiang): Initialize encoder.

    for (;;)
    {
      GetRemoteHeadSetPosition();
      std::cout << std::string(reinterpret_cast<const char*>(data_.get()),
                               msglen_)
                << std::endl;
      // TODO(cjiang): Set head position for graphic render engine
      // TODO(cjiang): Get rendered frame
      // TODO(cjiang): Encoded rendered frame
      usleep(1000);
      SendRenderedFrame(frame.get(), filesize);
    }
  }

 private:
  void GetRemoteHeadSetPosition()
  {
    boost::asio::read(socket_, boost::asio::buffer(&msglen_, kMSGLENGTHSIZE));
    std::cout << "data size " << msglen_ << std::endl;
    boost::asio::read(socket_, boost::asio::buffer(data_.get(), msglen_));
  }

  void SendRenderedFrame(uint8_t* frame, uint32_t size)
  {
    boost::asio::write(socket_, boost::asio::buffer(&size, kMSGLENGTHSIZE));
    boost::asio::write(socket_, boost::asio::buffer(frame, size));
    std::cout << "Server send bitstream: " << size << std::endl;
  }

 private:
  tcp::socket socket_;
  static const size_t kMSGLENGTHSIZE = 4;
  std::unique_ptr<uint8_t[]> data_;
  uint32_t msglen_;
};

class RenderServer
{
 public:
  RenderServer(boost::asio::io_service& io_service, unsigned short port)
      : io_service_(io_service)
      , signal_(io_service, SIGCHLD)
      , acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
      , socket_(io_service)
  {
    SignalWait();
    Accept();
  }

 private:
  void SignalWait()
  {
    signal_.async_wait(boost::bind(&RenderServer::SignalWaitHandler, this));
  }

  void SignalWaitHandler()
  {
    if (acceptor_.is_open())
    {
      int status = 0;
      while (waitpid(-1, &status, WNOHANG) > 0)
      {
      }

      SignalWait();
    }
  }

  void Accept()
  {
    acceptor_.async_accept(socket_,
                           boost::bind(&RenderServer::AcceptHandler, this, _1));
  }

  void AcceptHandler(const boost::system::error_code& ec)
  {
    if (!ec)
    {
      if (fork() == 0)  // child
      {
        io_service_.notify_fork(boost::asio::io_service::fork_child);
        acceptor_.close();
        signal_.cancel();
        socket_.set_option(tcp::no_delay(true));
        std::cout << "User Connected." << std::endl;
        std::make_shared<RenderEngine>(std::move(socket_))->Start();
      }
      else
      {
        io_service_.notify_fork(boost::asio::io_service::fork_parent);
        socket_.close();
        Accept();
      }
    }
    else
    {
      std::cerr << "Accept error: " << ec.message() << std::endl;
      Accept();
    }
  }

 private:
  boost::asio::io_service& io_service_;
  boost::asio::signal_set signal_;
  tcp::acceptor acceptor_;
  tcp::socket socket_;
};

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: process_per_connection <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;
    RenderServer s(io_service, std::atoi(argv[1]));
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}