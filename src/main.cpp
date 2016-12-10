#include "mbed.h"
#include "LogIt.h"
#include "LoggerInterface.h"

// #include "file_reader.h"
// #include "device_configuration.h"
// #include "json_device_configuration_parser.h"

#include "task_scheduler.h"
#include "alive.h"

#include "firmware_version.h"
#define DEBUG_MODE 1
#include "log.h"

#include "nrf24l01p_network.h"
#include "device_configuration.h"
#include "service_configuration.h"
#include "control_configuration.h"
#include "network_configuration.h"
#include "router.h"

// #include "on_off_control.h"
// #include "electronic_gate_control.h"

#include "client_factory.h"
#include "control_factory.h"
#include "network_factory.h"

#include "communicator.h"

extern "C" void mbed_reset();

Serial pc(USBTX, USBRX);
LogIt logger(&pc);

std::vector<HomeAutomator::EndPoint *> endPoints;

// void load_device_config(void) {
//   ConfigParser::FileReader reader("/local/basic.jsn");
//   JsonDeviceConfigurationParser parser(&reader);
//   DeviceConfiguration *devConfig = parser.parse();
//
//   logger.info("Firmware version = %s", devConfig->version.c_str());
//   logger.debug("Description = %s", devConfig->description.c_str());
//   logger.debug("Board = %s", devConfig->board.c_str());
//
//   delete devConfig;
// }

class NetworkJoin {
  private: HomeAutomator::Network * network;
  public: NetworkJoin(HomeAutomator::Network * network) { this->network = network; }
  public: void request_network_join(void) { this->network->request_network_join(); }
};

void create_services(HomeAutomator::Communicator * communicator, SimpleTaskScheduler::TaskScheduler * scheduler) {
    // TODO [DUPLICATION A1A]

    HomeAutomator::ServiceConfiguration * config = new HomeAutomator::ServiceConfiguration();
    if (!config) {
      SimplyLog::Log::e("Could not create ServiceConfiguration memory full\r\n");
      return;
    }
    config->load();

    std::vector<HomeAutomator::ServiceConfigData *> serviceConfig = config->get_service_configs();
    for (int i = 0; i < serviceConfig.size(); i++) {
      SimplyLog::Log::i("Creating service with id = %d, type = %s, #params: %d\r\n", serviceConfig[i]->id, serviceConfig[i]->type, serviceConfig[i]->params.size());
      HomeAutomator::EndPoint * service = HomeAutomator::ClientFactory::create(serviceConfig[i], communicator);
      if (service) {
        communicator->register_end_point(service);
        if (service->does_it_require_periodic_update()) {
          SimplyLog::Log::i("Creating scheduled update task (%f s) for service with id %d\r\n", service->get_periodic_update_time(), serviceConfig[i]->id);
          scheduler->create_periodic_task(service, &HomeAutomator::EndPoint::update, service->get_periodic_update_time());
        }
      }
    }
    delete config;
}

void create_controls(HomeAutomator::Communicator * communicator, SimpleTaskScheduler::TaskScheduler * scheduler) {
    // TODO [DUPLICATION A1A]

    HomeAutomator::ControlConfiguration * config = new HomeAutomator::ControlConfiguration();
    if (!config) {
      SimplyLog::Log::e("Could not create ControlConfiguration memory full\r\n");
      return;
    }
    config->load();

    std::vector<HomeAutomator::ControlConfigData *> controlConfig = config->get_control_configs();
    for (int i = 0; i < controlConfig.size(); i++) {
      SimplyLog::Log::i("Creating control with id = %d, type = %s, #params: %d\r\n", controlConfig[i]->id, controlConfig[i]->type, controlConfig[i]->params.size());
      HomeAutomator::EndPoint * control = HomeAutomator::ControlFactory::create(controlConfig[i], communicator);
      if (control) {
        communicator->register_end_point(control);
        if (control->does_it_require_periodic_update()) {
          SimplyLog::Log::i("Creating scheduled update task (%f s) for control with id %d\r\n", control->get_periodic_update_time(), controlConfig[i]->id);
          scheduler->create_periodic_task(control, &HomeAutomator::EndPoint::update, control->get_periodic_update_time());
        }
        endPoints.push_back(control);   // This is just for sending command through terminal, will disappear later
      }
    }
    delete config;
}

void create_networks(HomeAutomator::Router * router, SimpleTaskScheduler::TaskScheduler * scheduler) {
  HomeAutomator::NetworkConfiguration * config = new HomeAutomator::NetworkConfiguration();
  if (!config) {
    SimplyLog::Log::e("Could not create NetworkConfiguration memory full\r\n");
    return;
  }
  config->load();

  std::vector<HomeAutomator::NetworkConfigData *> networkConfigs = config->get_network_configs();
  for (int i = 0; i < networkConfigs.size(); i++) {
    SimplyLog::Log::i("Creating network of type %s with parent address = %d\r\n", networkConfigs[i]->type, networkConfigs[i]->parentAddress);

    HomeAutomator::Network * network = HomeAutomator::NetworkFactory::create(networkConfigs[i], router);

    if (!network) {
      SimplyLog::Log::e("Could not create %s network\r\n", networkConfigs[i]->type);
    } else {
      router->attach_network(network);
      SimplyLog::Log::d("%s network succesfully attached\r\n", networkConfigs[i]->type);

      // We need to create a task to do this as this should be done after
      // the network is fully set up. By retrying periodically we also
      // create simple retry mechanism.
      // TODO We should add these to a list so we can delete them later
      if (networkConfigs[i]->parentAddress != -1) {
        SimplyLog::Log::i("Creating periodic join request task\r\n");
        NetworkJoin * netJoin = new NetworkJoin(network);
        scheduler->create_periodic_task(netJoin, &NetworkJoin::request_network_join, 20);
      }
    }
  }
  delete config;
}

HomeAutomator::EndPoint * get_end_point_with_id(int id) {
  for (unsigned int i = 0; i < endPoints.size(); i++) {
    if (endPoints[i]->get_id() == id) {
      return endPoints[i];
    }
  }
  return NULL;
}

int main() {
  pc.baud(115200);

  logger.setLevel(Log::LoggerInterface::DEBUG);
  logger.info("Booting ....");

  // load_device_config();

  logger.debug("Creating task scheduler");
  SimpleTaskScheduler::TaskScheduler scheduler;

  logger.debug("Creating periodic alive task");
  Alive alive(LED1);
  scheduler.create_periodic_task(&alive, &Alive::indicate_living, 0.5);

  SimplyLog::Log::i("Loading config ...\r\n");
  HomeAutomator::DeviceConfiguration * deviceConfig = new HomeAutomator::DeviceConfiguration();
  if (!deviceConfig) {
    SimplyLog::Log::e("Could not create DeviceConfiguration, memory full\r\n");
    exit(1);
  }

  deviceConfig->load();

  SimplyLog::Log::i("ID: %s\r\n", deviceConfig->get_id());
  SimplyLog::Log::i("Description: %s\r\n", deviceConfig->get_description());
  SimplyLog::Log::i("Type: %s\r\n", deviceConfig->get_type());
  SimplyLog::Log::i("Address: %d\r\n", deviceConfig->get_address());

  SimplyLog::Log::d("Creating router with address = %d\r\n", deviceConfig->get_address());
  HomeAutomator::Router router(deviceConfig->get_address());
  HomeAutomator::Communicator communicator(&router);
  router.set_communicator(&communicator);

  create_networks(&router, &scheduler);
  create_services(&communicator, &scheduler);
  create_controls(&communicator, &scheduler);

  SimplyLog::Log::i("Creating continuous router task\r\n");
  scheduler.create_continuous_task(&router, &HomeAutomator::Router::check_for_packet);

  while(1) {
    // [TODO] Refactor:
    // This should later be refactored to a task
    if (pc.readable()) {
        char x = pc.getc();

        if (x == 'h') {
          logger.info("Press h for help");
          logger.info("Press r for reset");
          logger.info("Press s for network stats");
          logger.info("Press c for device config");
        } else if (x == 'r') {
            mbed_reset();
        } else if (x == 's') {
          SimplyLog::Log::i("Network stats:\r\n%s", router.get_stats());
          SimplyLog::Log::i("Network status:\r\n%s\r\n", router.to_string());
        } else if (x == 'c') {
          SimplyLog::Log::i("ID: %s\r\n", deviceConfig->get_id());
          SimplyLog::Log::i("Description: %s\r\n", deviceConfig->get_description());
          SimplyLog::Log::i("Type: %s\r\n", deviceConfig->get_type());
          SimplyLog::Log::i("Address: %d\r\n", deviceConfig->get_address());
        }
    }

    scheduler.update();
  }
}
