#include "program_controller.h"
namespace service {
ProgramController::ProgramController(const std::string& path, const std::vector<std::string>& args, int restart_limit)
    : path(path)
    , args(args)
    , restart_limit(restart_limit)
    , restart_count(0)
    , pid(-1)
{
}

bool ProgramController::start()
{
    pid = fork();
    if (pid == 0) {
        // 子进程执行程序
        std::vector<const char*> c_args;
        c_args.push_back(path.c_str());
        for (const auto& arg : args) {
            c_args.push_back(arg.c_str());
        }
        c_args.push_back(nullptr);
        execv(path.c_str(), const_cast<char* const*>(c_args.data()));
        std::cerr << "Failed to execv\n";
        exit(EXIT_FAILURE);
    } else if (pid > 0) {
        std::cout << "Program started with PID: " << pid << "\n";
        return true;
    } else {
        std::cerr << "Failed to fork process\n";
        return false;
    }
}

void ProgramController::stop()
{
    if (pid > 0) {
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        std::cout << "Program stopped\n";
        pid = -1;
    }
}

bool ProgramController::isRunning()
{
    if (pid <= 0)
        return false;
    return (kill(pid, 0) == 0);
}

bool ProgramController::restart()
{
    if (restart_count >= restart_limit) {
        std::cerr << "Restart limit reached for " << path << "\n";
        return false;
    }
    stop();
    restart_count++;
    return start();
}
}