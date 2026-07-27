#pragma once
#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>
namespace service {
class ProgramController {
public:
    ProgramController(const std::string& path, const std::vector<std::string>& args, int restart_limit);

    bool start();

    void stop();

    bool isRunning();

    bool restart();

private:
    std::string path;
    std::vector<std::string> args;
    int restart_limit;
    int restart_count;
    pid_t pid;
};
}
