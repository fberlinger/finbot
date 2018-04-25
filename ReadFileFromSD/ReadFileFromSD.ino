// ------------------------------------------------------
char fileName[ ] = "example.txt"; // adjust fileName here
// ------------------------------------------------------

#include <SPI.h>
#include <SD.h>

const int chipSelect = 8;

void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(9600);

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File blueBot = SD.open(fileName);

  // if the file is available, write to it:
  if (blueBot) {
    while (blueBot.available()) {
      Serial.write(blueBot.read());
    }
    blueBot.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
  } 
}

void loop()
{
}

