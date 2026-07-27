#pragma once
#include "program_controller.h"
#include <chrono>
#include <thread>
#include <vector>
namespace service {
class Monitor {
public:
    void addProgram(ProgramController* program);
    void startMonitoring();

    void stopMonitoring();

private:
    std::vector<ProgramController*> programs;
    bool monitoring = false;
    std::thread monitor_thread;
};
}
