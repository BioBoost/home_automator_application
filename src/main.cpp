#include "mbed.h"
#include "LogIt.h"
#include "LoggerInterface.h"

DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);        // tx, rx

int main() {
    pc.baud(115200);

    LogIt logger(&pc);
    logger.setLevel(Log::LoggerInterface::DEBUG);
    logger.debug("Booting ....");


    while(1) {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
    }
}
