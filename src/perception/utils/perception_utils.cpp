#include "perception_utils.h"

void initRosMsgFields(std::vector<sensor_msgs::msg::PointField>& fields) {
    fields.resize(4);
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = 7;
    fields[0].count = 1;
    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = 7;
    fields[1].count = 1;
    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = 7;
    fields[2].count = 1;
    fields[3].name = "intensity";
    fields[3].offset = 12;
    fields[3].datatype = 7;
    fields[3].count = 1;
}
   
void PCL2ROSMSg(const PointCloud::Ptr cloud_ptr, sensor_msgs::msg::PointCloud2& ros_msgs) {
    //set header with PCL Cloud header for common use. 
    // ros_msgs.header.stamp.fromNSec(cloud_ptr->header.stamp * 1000ull); // us => ns
    // ros_msgs.header.seq = cloud_ptr->header.seq;
    // ros_msgs.header.frame_id = cloud_ptr->header.frame_id;

    initRosMsgFields(ros_msgs.fields);

    ros_msgs.width = cloud_ptr->width;   // 1
    ros_msgs.height = cloud_ptr->height; // point_num
    ros_msgs.point_step = ros_msgs.fields.size() * sizeof(float); // 4 * 4
    ros_msgs.row_step   = static_cast<uint32_t> (ros_msgs.point_step * ros_msgs.width);
    ros_msgs.is_dense   = cloud_ptr->is_dense;

    int data_size = cloud_ptr->size() * ros_msgs.point_step;
    if (data_size > 0) {
        ros_msgs.data.resize(data_size);
        //memcpy(&ros_msgs.data[0], &cloud_ptr->points[0], data_size);
        float * pdata = reinterpret_cast<float*>(&ros_msgs.data[0]);
        for (int i = 0; i < cloud_ptr->size(); i++ ) {
            pcl::PointXYZ pt = cloud_ptr->points[i];
            pdata[i*4 + 0] = pt.x;
            pdata[i*4 + 1] = pt.y;
            pdata[i*4 + 2] = pt.z;
            pdata[i*4+ 3] = pt.z;
        }
    }
}

std::string getAbsolutePath(const std::string& relativePath) {
    return std::filesystem::absolute(relativePath).string();
}


std::string getProcessPath(void)
{
    char processDir[1000] = { 0 };

    if (readlink("/proc/self/exe", processDir, sizeof(processDir)) > 0) {
        auto end = strrchr(processDir, '/');
        if (end != nullptr) {
            *(end + 1) = '\0';
        }
    }
    return processDir;
}


std::string getLocalIPAddress() {
    struct ifaddrs *interfaces = nullptr;
    struct ifaddrs *ifa = nullptr;
    void *tmpAddrPtr = nullptr;
    std::string ipAddress = "";

    if (getifaddrs(&interfaces) == -1) return "";

    for (ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            if (std::string(addressBuffer) != "127.0.0.1") {
                ipAddress = std::string(addressBuffer);
                break;
            }
        }
    }

    freeifaddrs(interfaces);
    return ipAddress;
}

std::string get_mac_address(const std::string &interface_name) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return "";
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(fd, SIOCGIFHWADDR, &ifr) == -1) {
        perror("ioctl");
        close(fd);
        return "";
    }

    close(fd);

    unsigned char* mac = reinterpret_cast<unsigned char*>(ifr.ifr_hwaddr.sa_data);
    char mac_str[18];
    std::snprintf(mac_str, sizeof(mac_str),
                  "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(mac_str);
}

