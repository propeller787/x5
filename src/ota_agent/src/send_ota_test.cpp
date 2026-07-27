#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cstring>
#include <mosquitto.h>

#include "vehicle_ota_up.pb.h"

static void on_log(struct mosquitto*, void*, int level, const char* str){
    std::cerr << "[mosq] (" << level << ") " << (str?str:"") << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: send_ota_test <host> <port> <topic> [url=http://127.0.0.1:8000/dummy.bin] [ecu_id=ECU1] [version=1.2.3]\n";
        std::cerr << "Example: send_ota_test 127.0.0.1 1883 vehicle/ota/all/x5/1.0/up" << std::endl;
        return 1;
    }
    std::string host = argv[1];
    int port = std::stoi(argv[2]);
    std::string topic = argv[3];
    std::string url = (argc > 4) ? argv[4] : std::string("http://127.0.0.1:8000/dummy.bin");
    std::string ecu_id = (argc > 5) ? argv[5] : std::string("ECU1");
    std::string version = (argc > 6) ? argv[6] : std::string("1.2.3");

    // 构造最小可用的 VehicleOtaUp
    vehicle::ota::up::VehicleOtaUp up;
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    up.set_task_id("task_" + std::to_string(now_ms));
    up.set_task_status(0);
    up.set_timestamp(now_ms);
    up.set_exec_time(now_ms);
    up.mutable_strategy()->set_id("strategy_1");
    auto* pkg = up.mutable_strategy()->add_packages();
    pkg->set_package_id("pkg_" + std::to_string(now_ms));
    pkg->set_ecu(ecu_id);
    pkg->set_ecu_hardware_code("HW1");
    pkg->set_target_version(version);
    pkg->set_download_url(url);
    pkg->set_sha256("");
    pkg->set_size(0);

    std::string payload;
    if (!up.SerializeToString(&payload)) {
        std::cerr << "Serialize VehicleOtaUp failed" << std::endl;
        return 2;
    }

    mosquitto_lib_init();
    struct mosquitto* m = mosquitto_new(nullptr, true, nullptr);
    if (!m) {
        std::cerr << "mosquitto_new failed" << std::endl; return 3;
    }
    mosquitto_log_callback_set(m, on_log);
    int rc = mosquitto_connect(m, host.c_str(), port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "mosquitto_connect: " << mosquitto_strerror(rc) << std::endl;
        mosquitto_destroy(m);
        mosquitto_lib_cleanup();
        return 4;
    }
    rc = mosquitto_publish(m, nullptr, topic.c_str(), (int)payload.size(), payload.data(), 1, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "mosquitto_publish: " << mosquitto_strerror(rc) << std::endl;
        mosquitto_disconnect(m);
        mosquitto_destroy(m);
        mosquitto_lib_cleanup();
        return 5;
    }
    std::cout << "Published VehicleOtaUp to topic: " << topic << std::endl;
    mosquitto_disconnect(m);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();
    return 0;
}
