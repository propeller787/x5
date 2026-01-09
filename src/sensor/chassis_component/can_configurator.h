#pragma once

#include <string>

namespace sensor::chassis {

class CanConfigurator {
public:
    explicit CanConfigurator(std::string can);
    void setup();

private:
    std::string can_dev_;
};

}