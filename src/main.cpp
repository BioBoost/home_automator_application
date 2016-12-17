#include "mbed.h"
#include "serial_logger.h"
#include "LoggerInterface.h"

#include "file_reader.h"

#include "device_configuration.h"
#include "json_device_configuration_parser.h"

#include "endpoint_list_configuration.h"
#include "json_control_list_configuration_parser.h"
#include "json_client_list_configuration_parser.h"

#include "task_scheduler.h"
#include "alive.h"

#include "firmware_version.h"
#define DEBUG_MODE 1
#include "log.h"

#include "nrf24l01p_network.h"
#include "network_configuration.h"
#include "router.h"

// #include "on_off_control.h"
// #include "electronic_gate_control.h"

#include "client_factory.h"
#include "control_factory.h"
#include "network_factory.h"

#include "communicator.h"

extern "C" void mbed_reset();
extern int FreeMem();

Serial pc(USBTX, USBRX);
LogIt::SerialLogger logger(&pc);

std::vector<HomeAutomator::EndPoint *> endPoints;

class NetworkJoin {
  private: HomeAutomator::Network * network;
  public: NetworkJoin(HomeAutomator::Network * network) { this->network = network; }
  public: void request_network_join(void) { this->network->request_network_join(); }
};

HomeAutomator::EndPointListConfiguration * get_endpoint_configs(std::string configPath) {
  logger.debug("Loading endpoint configs: %s", configPath.c_str());
  ConfigParser::FileReader reader(configPath);
  HomeAutomator::JsonClientListConfigurationParser parser(&reader);
  HomeAutomator::EndPointListConfiguration * endPointConfigs = parser.parse();

  if (!endPointConfigs) {
    logger.error("Could not create endPointConfigs, memory full");
    return NULL;
  }

  return endPointConfigs;
}

void setup_endpoint(HomeAutomator::EndPoint * endpoint,
  HomeAutomator::Communicator * communicator,
  SimpleTaskScheduler::TaskScheduler * scheduler,
  HomeAutomator::EndPointConfiguration * endpointConfig) {

  communicator->register_end_point(endpoint);
  if (endpoint->does_it_require_periodic_update()) {
    logger.info("Creating scheduled update task (%f s) for endpoint with id %d", endpoint->get_periodic_update_time(), endpointConfig->id);
    scheduler->create_periodic_task(endpoint, &HomeAutomator::EndPoint::update, endpoint->get_periodic_update_time());
  }
  endPoints.push_back(endpoint);   // This is just for sending command through terminal, will disappear later
}

void create_clients(HomeAutomator::Communicator * communicator, SimpleTaskScheduler::TaskScheduler * scheduler) {
    HomeAutomator::EndPointListConfiguration * endPointConfigs = get_endpoint_configs("/local/clients.jsn");
    if (endPointConfigs) {
      for (int i = 0; i < endPointConfigs->endpoints.size(); i++) {
        HomeAutomator::EndPoint * client = HomeAutomator::ClientFactory::create((HomeAutomator::ClientConfiguration*)endPointConfigs->endpoints[i], communicator, &logger);
        if (client) {
          setup_endpoint(client, communicator, scheduler, endPointConfigs->endpoints[i]);
        }
      }
    }
    delete endPointConfigs;
}

void create_controls(HomeAutomator::Communicator * communicator, SimpleTaskScheduler::TaskScheduler * scheduler) {
    HomeAutomator::EndPointListConfiguration * endPointConfigs = get_endpoint_configs("/local/controls.jsn");
    if (endPointConfigs) {
      for (int i = 0; i < endPointConfigs->endpoints.size(); i++) {
        HomeAutomator::EndPoint * control = HomeAutomator::ControlFactory::create((HomeAutomator::ControlConfiguration*)endPointConfigs->endpoints[i], communicator, &logger);
        if (control) {
          setup_endpoint(control, communicator, scheduler, endPointConfigs->endpoints[i]);
        }
      }
    }
    delete endPointConfigs;
}

void create_networks(HomeAutomator::Router * router, SimpleTaskScheduler::TaskScheduler * scheduler) {
  HomeAutomator::NetworkConfiguration * config = new HomeAutomator::NetworkConfiguration();
  if (!config) {
    logger.error("Could not create NetworkConfiguration memory full");
    return;
  }
  config->load();

  std::vector<HomeAutomator::NetworkConfigData *> networkConfigs = config->get_network_configs();
  for (int i = 0; i < networkConfigs.size(); i++) {
    logger.info("Creating network of type %s with parent address = %d", networkConfigs[i]->type.c_str(), networkConfigs[i]->parentAddress);

    HomeAutomator::Network * network = HomeAutomator::NetworkFactory::create(networkConfigs[i], router, &logger);

    if (!network) {
      logger.error("Could not create %s network", networkConfigs[i]->type.c_str());
    } else {
      router->attach_network(network);
      logger.debug("%s network succesfully attached", networkConfigs[i]->type.c_str());

      // We need to create a task to do this as this should be done after
      // the network is fully set up. By retrying periodically we also
      // create simple retry mechanism.
      // TODO We should add these to a list so we can delete them later
      if (networkConfigs[i]->parentAddress != -1) {
        logger.info("Creating periodic join request task");
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
  logger.debug("Got %d memory free", FreeMem());

  logger.debug("Creating task scheduler");
  SimpleTaskScheduler::TaskScheduler scheduler;

  logger.debug("Creating periodic alive task");
  Alive alive(LED1);
  scheduler.create_periodic_task(&alive, &Alive::indicate_living, 0.5);

  logger.debug("Loading device config ...");
  ConfigParser::FileReader reader("/local/device.jsn");
  HomeAutomator::JsonDeviceConfigurationParser parser(&reader);
  HomeAutomator::DeviceConfiguration * deviceConfig = parser.parse();

  logger.info("ID: %s", deviceConfig->id.c_str());
  logger.info("Description: %s", deviceConfig->description.c_str());
  logger.info("Type: %s", deviceConfig->type.c_str());
  logger.info("Address: %d", deviceConfig->address);

  // delete deviceConfig;

  logger.debug("Creating router with address = %d", deviceConfig->address);
  HomeAutomator::Router router(deviceConfig->address, &logger);
  HomeAutomator::Communicator communicator(&router, &logger);
  router.set_communicator(&communicator);

  create_networks(&router, &scheduler);
  create_clients(&communicator, &scheduler);
  create_controls(&communicator, &scheduler);

  logger.debug("Creating continuous router task");
  scheduler.create_continuous_task(&router, &HomeAutomator::Router::check_for_frame);
  logger.debug("Got %d memory free", FreeMem());

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
          logger.info("Network stats: %s", router.get_stats().c_str());
          logger.info("Network status: %s", router.to_string().c_str());
        } else if (x == 'c') {
          logger.info("ID: %s", deviceConfig->id.c_str());
          logger.info("Description: %s", deviceConfig->description.c_str());
          logger.info("Type: %s", deviceConfig->type.c_str());
          logger.info("Address: %d", deviceConfig->address);
        }
    }

    scheduler.update();
  }
}
