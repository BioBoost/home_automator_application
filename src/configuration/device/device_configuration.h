#pragma once

#include <string>
#include "configuration.h"

class DeviceConfiguration : public ConfigParser::Configuration {

  public:
    std::string version;
    std::string description;
    std::string board;

  public:
    DeviceConfiguration() {
      version = "1.0";
      description = "DEADBEEF";
      board = "FRDM-KL25Z ";
    }

};
