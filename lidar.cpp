#include "lidar.h"

bool LiDAR::begin() {
    Serial.begin(115200);
    delay(20);

    printf("\n");
    printf("TFMPlus I2C Library Example - 14JAN2022\n\n");

    // Set up I2C once 
    Wire.begin(PIN_WIRE1_SDA, PIN_WIRE1_SCL);  // SDA=21, SCL=22
    Wire.setClock(400000);

    // Optionally recover the bus once after init
    tfmP.recoverI2CBus(PIN_WIRE1_SDA, PIN_WIRE1_SCL);

    printf("System reset: ");
    if (tfmP.sendCommand(SOFT_RESET, 0)) {
        printf("passed.\r\n");
    } else {
        tfmP.printReply();
    }

    printf("Firmware version: ");
    if (tfmP.sendCommand(GET_FIRMWARE_VERSION, 0)) {
        printf("%1u.", tfmP.version[0]);
        printf("%1u.", tfmP.version[1]);
        printf("%1u\n", tfmP.version[2]);
    } else {
        tfmP.printReply();
    }

    printf("Data-Frame rate: ");
    if (tfmP.sendCommand(SET_FRAME_RATE, FRAME_1000)) {
        printf("%2uHz.\n", FRAME_1000);
    } else {
        tfmP.printReply();
    }

    delay(500);
    return true;
}

bool LiDAR::getData(float &tfDist, int16_t &tfFlux, int16_t &tfTemp) {
    // I2C is already set up in begin(), so no Wire.begin() here
    tfmP.getData(tfDist, tfFlux, tfTemp);

    if (tfmP.status == TFMP_READY) {
        // Optional debug
        /*
        printf("Dist:%04icm ", tfDist);
        printf("Flux:%05i ", tfFlux);
        printf("Temp:%2i%s", tfTemp, "°C");
        printf("\n");
        */
    } else {
        tfmP.printFrame();
        if (tfmP.status == TFMP_I2CWRITE) {
            tfmP.recoverI2CBus(PIN_WIRE1_SDA, PIN_WIRE1_SCL);
        }
    }
    return true;
}
