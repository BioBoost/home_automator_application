#pragma once

#include "mbed.h"

class Alive {
  private: DigitalOut aliveLed;
  public: Alive(PinName led) : aliveLed(led) {}
  public: void indicate_living(void) { aliveLed = !aliveLed; }
};
