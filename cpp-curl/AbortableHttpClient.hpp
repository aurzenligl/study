#pragma once

#include <jsonrpccpp/client/connectors/httpclient.h>

namespace mocker
{

class AbortableHttpClient : public jsonrpc::HttpClient
{
public:
    explicit AbortableHttpClient(const std::string& url);
    void SendRPCMessage(const std::string& message, std::string& result) throw (jsonrpc::JsonRpcException) override;
    void abort();

private:
    static int progressCallback(void* clientp, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);

    CURL* curl_;
    bool abort_;
};

}  // namespace mocker
