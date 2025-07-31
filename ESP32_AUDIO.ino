#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Audio.h>

// WiFi credentials
const char* ssid = "TELLO";
const char* password = "12345678";

// I2S pin configuration
#define I2S_DOUT 25
#define I2S_BCLK 27
#define I2S_LRC  26

Audio audio;
WebServer server(80);
String currentTextToSpeak = "";
bool isSpeaking = false;
unsigned long speechStartTime = 0;
const unsigned long MAX_SPEAK_DURATION = 4000; // 4 seconds max per message

void handleSpokenText() {
  if (server.hasArg("text")) {
    if (!isSpeaking) {
      currentTextToSpeak = server.arg("text");
      Serial.print("Received text: ");
      Serial.println(currentTextToSpeak);
      server.send(200, "text/plain", "Accepted: Speaking once");
    } else {
      server.send(429, "text/plain", "Busy: Already speaking");
      Serial.println("Rejected: Still speaking");
    }
  } else {
    server.send(400, "text/plain", "Error: 'text' argument not found");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.println(F("Connecting to WiFi..."));
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 Speaker IP: ");
  Serial.println(WiFi.localIP());

  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  server.on("/speak", HTTP_GET, handleSpokenText);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  audio.loop(); // Keep the audio running

  // Start speaking
  if (!isSpeaking && currentTextToSpeak != "") {
    playText(currentTextToSpeak);
    speechStartTime = millis();
    isSpeaking = true;
    currentTextToSpeak = "";  // Clear after sending
  }

  // Stop after natural finish OR timeout
  if (isSpeaking) {
    if (!audio.isRunning() || (millis() - speechStartTime > MAX_SPEAK_DURATION)) {
      isSpeaking = false;
      Serial.println("Finished speaking (or forced timeout)");
    }
  }
}

void playText(const String& textToSpeak) {
  String chunk = textToSpeak;
  chunk.replace(" ", "%20");
  String tts_url = "http://translate.google.com/translate_tts?ie=UTF-8&q=" + chunk + "&tl=en&client=tw-ob";

  Serial.print("TTS URL: ");
  Serial.println(tts_url);

  audio.connecttohost(tts_url.c_str());
}