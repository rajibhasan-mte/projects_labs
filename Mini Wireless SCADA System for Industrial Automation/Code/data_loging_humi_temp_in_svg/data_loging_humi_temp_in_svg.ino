#include <SPI.h>
#include <SD.h>
#include "DHT.h"

// --- User Config ---
#define DHTPIN 5
#define DHTTYPE DHT11
#define SD_CS_PIN 14

DHT dht(DHTPIN, DHTTYPE);
const char* svgFileName = "/temperature_humidity.svg";

void setup() {
  Serial.begin(115200);
  delay(500);

  // Init DHT
  dht.begin();

  // Init SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed!");
    while (true) delay(500);
  }
  Serial.println("SD Card Initialized.");

  // If file doesn't exist, create and write SVG header
  if (!SD.exists(svgFileName)) {
    File file = SD.open(svgFileName, FILE_WRITE);
    if (file) {
      file.println("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>");
      file.println("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"800\" height=\"400\">");
      file.println("<text x=\"40\" y=\"40\" font-family=\"Arial\" font-size=\"28\" fill=\"#222\">Temperature & Humidity Log</text>");
      file.println("</svg>"); // Will update later, we append before this
      file.close();
    }
  }
}

void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read DHT sensor!");
    delay(2000);
    return;
  }

  Serial.print("Temp: "); Serial.print(temperature); Serial.print(" °C  ");
  Serial.print("Humidity: "); Serial.println(humidity);

  // Read existing SVG content
  File file = SD.open(svgFileName, FILE_READ);
  String svgContent;
  if (file) {
    while (file.available()) {
      svgContent += char(file.read());
    }
    file.close();
  }

  // Remove the closing </svg> tag
  int svgEnd = svgContent.lastIndexOf("</svg>");
  if (svgEnd > 0) {
    svgContent = svgContent.substring(0, svgEnd);
  }

  // Add new reading as text element
  String textLine = "<text x=\"40\" y=\"" + String(80 + (millis()/2000 % 300)) +
                    "\" font-family=\"Arial\" font-size=\"20\" fill=\"#d9534f\">"
                    "Temp: " + String(temperature,1) + "°C, Humidity: " +
                    String(humidity,1) + "%</text>\n";

  svgContent += textLine;

  // Close SVG
  svgContent += "</svg>\n";

  // Write back to SD card
  file = SD.open(svgFileName, FILE_WRITE);
  if (file) {
    file.print(svgContent);
    file.close();
    Serial.println("SVG updated successfully.");
  } else {
    Serial.println("Failed to write SVG file.");
  }

  delay(2000); // DHT11 read interval
}
