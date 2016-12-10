#pragma once

#include "json_configuration_parser.h"

class JsonDeviceConfigurationParser : ConfigParser::JsonConfigurationParser {

  public:
    JsonDeviceConfigurationParser(ConfigParser::Reader * reader)
      : ConfigParser::JsonConfigurationParser(reader) { }

  public:
    DeviceConfiguration * parse(void) {
      DeviceConfiguration * devConfig = new DeviceConfiguration();

      if (json.hasMember("version")) {
        devConfig->version = this->json["version"].get<std::string>();
      }

      if (json.hasMember("description")) {
        devConfig->description = this->json["description"].get<std::string>();
      }

      if (json.hasMember("board")) {
        devConfig->board = this->json["board"].get<std::string>();
      }

      return devConfig;
    }
};
