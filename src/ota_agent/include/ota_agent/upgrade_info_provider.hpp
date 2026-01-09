#pragma once
#include <string>
#include <optional>

namespace ota_agent {

// Decoupled data source interface for upgrade information.
// Implementations may fetch from cloud APIs, local files, or sensors.
class UpgradeInfoProvider {
public:
    virtual ~UpgradeInfoProvider() = default;

    // Current software version
    // Returns true on success; fills out fields accordingly.
    virtual bool get_current_version(std::string& current_version,
                                     int& code,
                                     std::string& message) = 0;

    // Upgrade content description (ECU/what is being upgraded)
    virtual bool get_upgrade_content(std::string& ecu_content,
                                     int& code,
                                     std::string& message) = 0;

    // Package size in bytes
    virtual bool get_package_size(double& package_size,
                                  int& code,
                                  std::string& message) = 0;

    // Whether the vehicle is currently upgradable (0=upgradable, 1=not upgradable)
    virtual std::optional<int> get_upgradable_status() = 0;
};

} // namespace ota_agent
