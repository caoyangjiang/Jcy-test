#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <cstdlib>
#include <iostream>
#include <memory>

using boost::asio::ip::tcp;

class RenderClient
{
 public:
  RenderClient& operator=(const RenderClient&) = delete;
  RenderClient(const RenderClient&)            = delete;

  explicit RenderClient(boost::asio::io_service& io_service,
                        tcp::resolver::iterator endpoint_iterator)
      : socket_(io_service)
  {
    boost::asio::connect(socket_, endpoint_iterator);
    Start();
  }

  void Start()
  {
    // TODO(cjiang): Initialize graphic rendering
    // TODO(cjiang): Initialize decoder.
    buffer_ = std::unique_ptr<uint8_t[]>(new uint8_t[4 * 1024 * 1024]);

    for (;;)
    {
      int next;
      // TODO(cjiang): Get head position from headset and fill data_ and msglen
      SendHeadSetPosition(reinterpret_cast<uint8_t*>(&next), 4);
      ReadCodedBitStream();
      // TODO(cjiang): Give received bitstream to decoder
      // TODO(cjiang): decodes the frame
      //
      // std::cin >> next;
    }
  }

 private:
  void SendHeadSetPosition(uint8_t* data, int size)
  {
    boost::asio::write(socket_, boost::asio::buffer(&size, kMSGLENGTHSIZE));
    boost::asio::write(socket_, boost::asio::buffer(data, size));
    // std::cout << "Client: headset position: " << *data << std::endl;
  }

  void ReadCodedBitStream()
  {
    boost::asio::read(socket_, boost::asio::buffer(&msglen_, kMSGLENGTHSIZE));
    boost::asio::read(socket_, boost::asio::buffer(buffer_.get(), msglen_));
    // std::cout << "Client: receive bitstream: " << msglen_ << std::endl;
  }

 private:
  tcp::socket socket_;
  static const size_t kMSGLENGTHSIZE = 4;
  std::unique_ptr<uint8_t[]> buffer_;
  uint32_t msglen_;
};

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage:  <address> <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;
    tcp::resolver resolver(io_service);
    auto iterator = resolver.resolve({argv[1], argv[2]});
    RenderClient client(io_service, iterator);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}