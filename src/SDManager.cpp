#include "SDManager.h"
#include "Macro.h"

void SDManager::processSdRecord(SDData &sdReceived,bool &isFileOpen)
{
    if (sdReceived.isMeasuring)
    {
        if (!isFileOpen)
        {
            isFileOpen = createFile(sdReceived);
        }

        if (isFileOpen)
        {
            updateSD(sdReceived);
        }
    }
    else if (isFileOpen)
    {
        if (closeFile())
        {
            isFileOpen = false;
        }
    }
}

void SDManager::updateSD(SDData sdReceived)
{
  unsigned int hours = sdReceived.timeMs / 3600000;
  unsigned int minutes = (sdReceived.timeMs / 60000) % 60;
  unsigned int seconds = (sdReceived.timeMs / 1000) % 60;
  unsigned int milliseconds = sdReceived.timeMs % 1000;

  char line[256];

  snprintf(
      line,
      sizeof(line),
      "%03u %02u%02u%02u.%03u "
      "%+014.8f %+014.8f "
      "%07.3f %07.3f "
      "%+09.2f %+08.2f "
      "%.3f\r\n",

      static_cast<unsigned int>(sdReceived.siv),
      hours,
      minutes,
      seconds,
      milliseconds,
      sdReceived.lat,
      sdReceived.lon,
      sdReceived.velocityKmh,
      sdReceived.headingDeg,
      sdReceived.heightM,
      sdReceived.verticalVelocity,
      sdReceived.samplePeriodS);

  file.print(line);
}

bool SDManager::begin()
{
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(SD_MOSI, OUTPUT);
  pinMode(SD_MISO, INPUT);
  pinMode(SD_SCK, OUTPUT);

  softSpi.begin();

  SdSpiConfig config(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi);

  Serial.print("[SD] detect pin = ");
  Serial.println(digitalRead(SD_DETECT_PIN));

  if (!sd.begin(config))
  {
    Serial.println("[SD] begin failed");
    sd.initErrorPrint(&Serial);
    return false;
  }

  Serial.println("[SD] begin OK");
  return true;
}

bool SDManager::createFile(SDData sdReceived)
{
  char title[32];

  snprintf(
      title,
      sizeof(title),
      "ApexIO-%02d-%02d.vbo",
      sdReceived.hr,
      sdReceived.min);

  if (!file.open(title, O_WRONLY | O_CREAT | O_TRUNC))
    return false;

  char banner[80];

  snprintf(
      banner,
      sizeof(banner),
      "ApexIO .vbo file created at %02u:%02u:%02u\r\n\r\n",
      static_cast<unsigned int>(sdReceived.timeMs / 3600000),
      static_cast<unsigned int>((sdReceived.timeMs / 60000) % 60),
      static_cast<unsigned int>((sdReceived.timeMs / 1000) % 60));

  file.print(banner);

  file.print(
      "[header]\r\n"
      "satellites\r\n"
      "time\r\n"
      "latitude\r\n"
      "longitude\r\n"
      "velocity kmh\r\n"
      "heading\r\n"
      "height\r\n"
      "vertical velocity m/s\r\n"
      "sampleperiod\r\n"
      "[channel units]\r\n"
      "[comments]\r\n"
      "[laptiming]\r\n"
      "[circuit details]\r\n"
      "[session data]\r\n"
      "[column names]\r\n"
      "sats time lat long velocity heading height vert-vel Tsample\r\n"
      "[data]\r\n");

  return true;
}

bool SDManager::closeFile()
{
  return file.close();
}

