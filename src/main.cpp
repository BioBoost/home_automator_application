#include "mbed.h"
#include "LogIt.h"
#include "LoggerInterface.h"

#include "file_reader.h"
#include "device_configuration.h"
#include "json_device_configuration_parser.h"

#include "task_scheduler.h"
#include "alive.h"

extern "C" void mbed_reset();

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

  logger.debug("Creating task scheduler");
  SimpleTaskScheduler::TaskScheduler scheduler;

  logger.debug("Creating periodic alive task");
  Alive alive(LED1);
  scheduler.create_periodic_task(&alive, &Alive::indicate_living, 0.5);

  while(1) {
    // [TODO] Refactor:
    // This should later be refactored to a task
    if (pc.readable()) {
        char x = pc.getc();

        if (x == 'h') {
          logger.info("Press h for help");
          logger.info("Press r for reset");
        } else if (x == 'r') {
            mbed_reset();
        }
    }

    scheduler.update();
  }
}
