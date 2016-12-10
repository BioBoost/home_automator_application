#include "mbed.h"
#include "LogIt.h"
#include "LoggerInterface.h"

#include "file_reader.h"
#include "device_configuration.h"
#include "json_device_configuration_parser.h"

DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);
LogIt logger(&pc);

void load_device_config(void) {
  ConfigParser::FileReader reader("/local/device.jsn");
  JsonDeviceConfigurationParser parser(&reader);
  DeviceConfiguration *devConfig = parser.parse();

  logger.info("Firmware version = %s", devConfig->version.c_str());
  logger.debug("Description = %s", devConfig->description.c_str());
  logger.debug("Board = %s", devConfig->board.c_str());

  delete devConfig;
}

int main() {
    pc.baud(115200);

    logger.setLevel(Log::LoggerInterface::DEBUG);
    logger.info("Booting ....");

    load_device_config();

    while(1) {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
    }
}
