// Copyright @ 2016 Caoyang Jiang

#include <curl/curl.h>
#include <fstream>
#include <iostream>
#include <memory>
class DLData
{
 public:
  std::unique_ptr<uint8_t[]> buffer_;
  size_t buffersize_ = 0;
  size_t bytecount_  = 0;
};

size_t CurlReceiveBlockCallBack(void *contents,
                                size_t size,
                                size_t nmemb,
                                void *userdata)
{
  DLData *dldata = reinterpret_cast<DLData *>(userdata);
  uint8_t *bytes = reinterpret_cast<uint8_t *>(contents);

  for (size_t i = 0; i < size * nmemb; i++)
  {
    dldata->buffer_[dldata->bytecount_] = bytes[i];
    dldata->bytecount_++;
  }

  return size * nmemb;
}

int main()
{
  DLData dldata;
  CURL *curl;
  CURLcode res;

  curl_global_init(CURL_GLOBAL_DEFAULT);
  curl = curl_easy_init();
  curl_easy_setopt(curl,
                   CURLOPT_URL,
                   "http://yt-dash-mse-test.commondatastorage.googleapis.com/"
                   "media/car-20120827-manifest.mpd");

  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlReceiveBlockCallBack);

  dldata.buffer_     = std::unique_ptr<uint8_t[]>(new uint8_t[10000]);
  dldata.buffersize_ = 10000;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, reinterpret_cast<void *>(&dldata));
  curl_easy_setopt(curl, CURLOPT_RANGE, "0-199");
  res = curl_easy_perform(curl);

  if (res != CURLE_OK)
  {
    std::cout << "Curl_easy_perform() failed: " << curl_easy_strerror(res)
              << std::endl;
    return 1;
  }
  curl_easy_cleanup(curl);
  curl_global_cleanup();
  std::cout << dldata.bytecount_ << std::endl;
  return 0;
}
