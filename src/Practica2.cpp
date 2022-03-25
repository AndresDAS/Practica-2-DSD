#include <Arduino.h>
#include <UbiConstants.h>
#include <UbidotsEsp32Mqtt.h>
#include <UbiTypes.h>
#include "UbidotsEsp32Mqtt.h"
#include <cstring>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define DHTPIN 27
#define DHTTYPE DHT11

const char *UBIDOTS_TOKEN = "BBFF-co4IS9ILXqSDZogZGTAC3DMUnyAt9s"; // Put here your Ubidots TOKEN
const char *WIFI_SSID = "LIBARDO SALDARRIAgA";
const char *WIFI_PASS = "libardo302";
const char *DEVICE_LABEL = "esp32"; // Replace with the device label to subscribe to
const char *VARIABLE_LABEL = "SW1"; // Replace with your variable label to subscribe to
const char *VARIABLE_LABEL2 = "SW2";
const char *VARIABLE_LABEL3 = "temp";
const char *VARIABLE_LABEL4 = "hum";

const uint8_t LED1 = 32; // Pin used to write data based on 1's and 0's coming from Ubidots
const uint8_t LED2 = 33;

char msg1[] = "/v2.0/devices/esp32/sw1/lv";
char msg2[] = "/v2.0/devices/esp32/sw2/lv";

const int PUBLISH_FREQUENCY = 5000;

int cmp1 = 1, cmp2 = 1;
int pc1x = 40, pc2x = 100, pcy = 205, r = 20;

unsigned long timer;

int estadoled1 = 0, estadoled2 = 0;

TFT_eSPI tft = TFT_eSPI();

DHT dht(DHTPIN, DHTTYPE);

Ubidots ubidots(UBIDOTS_TOKEN);

// Funciones

void callback(char *topic, byte *payload, unsigned int length);
void setup();
void loop();

// Rutinas

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Cambios de estados
  cmp1 = strcmp(topic, msg1);
  cmp2 = strcmp(topic, msg2);

  if (cmp1 == 0)
  {
    if ((char)payload[0] == '0')
    {
      estadoled1 = 0;
      Serial.println("LED 1 off");
    }
    if ((char)payload[0] == '1')
    {
      estadoled1 = 1;
      Serial.println("LED 1 on");
    }
  }

  if (cmp2 == 0)
  {
    if ((char)payload[0] == '0')
    {
      estadoled2 = 0;
      Serial.println("LED 2 off");
    }
    if ((char)payload[0] == '1')
    {
      estadoled2 = 1;
      Serial.println("LED 2 on");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  tft.init();
  tft.setTextColor(TFT_WHITE);
  tft.fillScreen(TFT_BLACK); // Color de pantalla
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Inicializacion DHT
  Serial.println(F("DHTxx test!"));
  dht.begin();

  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL); // Insert the dataSource and Variable's Labels
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL2);

  timer = millis();
}

void loop()
{
  delay(500);
  // put your main code here, to run repeatedly:
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    tft.fillScreen(TFT_RED);
    tft.drawString("Failed to read from", 10, 20, 2);
    tft.drawString("DHT sensor!", 10, 40, 2);
    return;
  }

  tft.fillScreen(TFT_BLACK);
  tft.drawString("Humedad (%): ", 10, 10, 2);
  tft.drawString(String(h), 0, 40, 7);
  tft.drawString("Temperatura (°C): ", 10, 100, 2);
  tft.drawString(String(t), 0, 130, 7);

  // Encendido y Apagado de LEDs
  if (estadoled1 == 1 && estadoled2 == 1)
  {
    tft.fillCircle(pc1x, pcy, r, TFT_RED);
    tft.fillCircle(pc2x, pcy, r, TFT_GREEN);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
  }
  if (estadoled1 == 1 && estadoled2 == 0)
  {
    tft.fillCircle(pc1x, pcy, r, TFT_RED);
    tft.drawCircle(pc2x, pcy, r, TFT_GREEN);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
  }
  if (estadoled1 == 0 && estadoled2 == 1)
  {
    tft.drawCircle(pc1x, pcy, r, TFT_RED);
    tft.fillCircle(pc2x, pcy, r, TFT_GREEN);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
  }
  if (estadoled1 == 0 && estadoled2 == 0)
  {
    tft.drawCircle(pc1x, pcy, r, TFT_RED);
    tft.drawCircle(pc2x, pcy, r, TFT_GREEN);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }

  if (!ubidots.connected())
  {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL);
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL2);
  }
  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {

    ubidots.add(VARIABLE_LABEL3, t); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL4, h);
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  ubidots.loop();
}
