#include "AbortableHttpClient.hpp"

namespace mocker
{

struct HttpClientData : public jsonrpc::IClientConnector
{
    std::map<std::string,std::string> headers;
    std::string url;
    long timeout;
    CURL* curl;
};

static_assert(sizeof(jsonrpc::HttpClient) == sizeof(HttpClientData), "size of jsonrpc::HttpClient doesn't match with HttpClientData");

static CURL* getCurl(AbortableHttpClient* client)
{
    // Data model of jsonrpc::HttpClient had to be redefined in order to access CURL* data.
    // It's due to this data being declared as a private member of class, instead of protected.
    jsonrpc::HttpClient* httpClient = static_cast<jsonrpc::HttpClient*>(client);
    HttpClientData* httpClientData = static_cast<HttpClientData*>(static_cast<void*>(httpClient));
    return httpClientData->curl;
}

AbortableHttpClient::AbortableHttpClient(const std::string& url):
    jsonrpc::HttpClient(url),
    curl_(getCurl(this)),
    abort_(false)
{}

void AbortableHttpClient::SendRPCMessage(const std::string& message, std::string& result) throw (jsonrpc::JsonRpcException)
{
    curl_easy_setopt(curl_, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl_, CURLOPT_XFERINFODATA, this);
    curl_easy_setopt(curl_, CURLOPT_XFERINFOFUNCTION, progressCallback);
    // CURLOPT_OPENSOCKETFUNCTION & CURLOPT_OPENSOCKETDATA can also be used to get access to socket
    // this can allow faster shutdown by shutting down socket directly

    jsonrpc::HttpClient::SendRPCMessage(message, result);
}

void AbortableHttpClient::abort()
{
    abort_ = true;
}

int AbortableHttpClient::progressCallback(void* clientp, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow)
{
    AbortableHttpClient& self = *static_cast<AbortableHttpClient*>(clientp);
    return self.abort_;
}

}  // namespace mocker
