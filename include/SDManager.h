#pragma once

#include <Arduino.h>
#include <TelemetryData.h>
#include <SdFat.h>

#include "Macro.h"

class SDManager
{
private:
    SoftSpiDriver<SD_MISO, SD_MOSI, SD_SCK> softSpi;
    SdFat sd;
    SdFile file;

public:
    void processSdRecord(SDData &sdReceived,bool &isFileOpen);
    bool begin();
    void updateSD(SDData sdReceived);
    bool createFile(SDData sdReceived);
    bool closeFile();

    bool isFileOpen() const
    {
        return file.isOpen();
    }

    bool isCardPresent() const
    {
        return digitalRead(SD_DETECT_PIN) == HIGH;
    }
};