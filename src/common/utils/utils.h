#pragma once

#include "string"

#include <list>

namespace utils {

extern double legalizeAngle(double angle);
extern double angleDiff(double a, double b);
extern double angleAverage(std::list<double> angles);
extern uint64_t format_time();
}