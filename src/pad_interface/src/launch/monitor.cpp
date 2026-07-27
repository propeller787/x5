#include "monitor.h"

namespace service {

void Monitor::addProgram(ProgramController* program)
{
    programs.push_back(program);
}

void Monitor::startMonitoring()
{
    monitoring = true;
    monitor_thread = std::thread([this]() {
        while (monitoring) {
            for (auto program : programs) {
                if (!program->isRunning()) {
                    std::cout << "Program crashed, restarting...\n";
                    program->restart();
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(5)); // 每 5 秒检查一次
        }
    });
}

void Monitor::stopMonitoring()
{
    monitoring = false;
    if (monitor_thread.joinable()) {
        monitor_thread.join();
    }
}

}