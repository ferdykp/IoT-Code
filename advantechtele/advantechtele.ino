#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include "DHT.h"
#include <ArduinoJson.h>

#define DHTPIN D1

char ssid[] = "?"; //nama wifi
char password[] = "ferdykurnia"; //password wifi

// Initialize Telegram BOT
#define BOTtoken "6298927547:AAHPbRMyVbTLfkrBjtPtUBsVSgzXbwkfHmE"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define idChat "1418915527"

X509List cert(TELEGRAM_CERTIFICATE_ROOT);

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

#define DHTTYPE DHT11
DHT dht(D1, DHTTYPE);

void setup() {
  Serial.begin(115200);
  
    configTime(0, 0, "pool.ntp.org");      // get UTC time via NTP
    client.setTrustAnchors(&cert); // Add root certificate for api.telegram.org
  
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  dht.begin();
  delay(5000);
}

void loop() {
  delay(2000);
  
  float t = dht.readTemperature();
  Serial.print("Suhu saat ini : ");
  Serial.print(t);
  Serial.println(" *C");
  
  if (t > 30.00) {
    bot.sendChatAction(idChat, "Sedang mengetik...");
    Serial.print("Suhu saat ini : ");
    Serial.println(t);
    delay(3000);

    String suhu = "Intensitas suhu : ";
    suhu += int(t);
    suhu += " *C\n";
    suhu += "Suhu maksimal gaes!\n";
    bot.sendMessage(idChat, suhu, "");
    Serial.print("Mengirim data sensor ke telegram");
  }
}
