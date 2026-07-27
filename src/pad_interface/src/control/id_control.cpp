#include "id_control.h"
#include <vector>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <random>
namespace service::control {

static int read_mec_sn(char* sn)
{
    if (!sn) {
        return -1;
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return -1;
    }

    struct ifreq ifreq;
    strcpy(ifreq.ifr_name, "eth0");
    if (ioctl(sock, SIOCGIFHWADDR, &ifreq) < 0) {
        return -1;
    }

    int byteCount = 6;
    for (int i = 0; i < byteCount; i++) {
        sprintf(sn + 2 * i, "%02X", (unsigned char)ifreq.ifr_hwaddr.sa_data[i]);
    }

    return 2 * byteCount;
}

IdControl::IdControl()
    : Control("id")
{
    char snbuf[256];
    int id = read_mec_sn(snbuf);  // 通过data()获取底层数组指针
    auto sn = id > 0 ? std::string(snbuf, id) : "Unknown";
    id_ = static_cast<int>(std::stoul(sn, nullptr, 16));
    auto global = service::Global::getInstance();
    id_controls_ = global->node_->create_service<interface::srv::IdControls>(idControlService, std::bind(&IdControl::handleIdControls, this, _1, _2));
}

IdControl::~IdControl()
{
}

void IdControl::handleIdControls(const std::shared_ptr<interface::srv::IdControls::Request> request,
    std::shared_ptr<interface::srv::IdControls::Response> response)
{
    response->set__sn(std::to_string(id_));
}

}
