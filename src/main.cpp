#include <WiFi.h>
#include <PubSubClient.h>
#include <cmath>
#include <ArduinoJson.h>

#define RXD2 16                          // RXD2 do ESP32 conectado ao TX do GPS
#define TXD2 17                          // TXD2 do ESP32 conectado ao RX do GPS
#define GPS_BAUDRATE 9600                // Baud rate do GPS
#define WIFI_SSID "redewifi"             // SSID da rede WiFi
#define WIFI_PASSWORD "senhawifi"        // Senha da rede WiFi
#define MQTT_SERVER "broker.emqx.io"     // Endereço do servidor MQTT
#define MQTT_PORT 1883                   // Porta do servidor MQTT
#define MQTT_TOPIC_PREFIX "data/checker" // Préfixo do tópico MQTT para publicar as coordenadas
#define MQTT_USERNAME "emqx"             // Usuario do servidor
#define MQTT_PASSWORD "public"           // Senha do usuario
#define LED 2                            // Led interno do esp32
#define LED_EXT 15                       // Led externo ao esp32

const double LATITUDE_REFERENCE = -5.77179;                        // Latitude de referência para calcular o distanciamento do dispositivo
const double LONGITUDE_REFERENCE = -37.56939;                      // Longitude de referência para calcular o distanciamento do dispositivo
const double MAX_DISTANCE = 100;                                   // Distância limite usada para disparar SMS de alerta de distanciamento, medida em metros
const String TOPIC_DRONE = "data/tracker/esp32-00:00:00:00:00:00"; // Tópico que retorna a localização atual do drone

WiFiClient wifiClient;           // Cliente WiFi
PubSubClient client(wifiClient); // Cliente MQTT
StaticJsonDocument<192> doc;     // Criação do documento (objeto que vai ajudar a calcular a distancia)
StaticJsonDocument<256> doc2;    // Criação do documento (objeto que vai ser publicado)

// Função responsável por conectar ao Wifi
void setup_wifi()
{
  delay(1000);

  Serial.println();
  Serial.print("Conectando ao wifi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Enquanto nao for estabelecida, fica aguardadando
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  // Conexao com wifi estabelecida. Aciona o led interno
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED, HIGH);
}

// Função responsárvel por conectar ao MQTT e realizar o subscribe referente ao tópico do drone
void connectToMqtt()
{
  while (!client.connected())
  {
    String client_id = "esp32-";
    client_id += String(WiFi.macAddress());
    Serial.println("Conectando ao broker MQTT...");
    if (client.connect(client_id.c_str(), MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("Conectado ao broker MQTT!");
      if (client.subscribe(TOPIC_DRONE.c_str()))
      {
        Serial.println("Subscribed");
      }
    }
    else
    {
      Serial.print("Falha na conexão MQTT: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// Calcula a distância em que o dispositivo está, levando em consideração a curvatura da terra (valor retornado em metros) formula de vicenty
double vincenty(double lat1, double lon1, double lat2, double lon2)
{
  const double a = 6378137;           // semieixo maior da Terra em metros
  const double b = 6356752.314245;    // semieixo menor da Terra em metros
  const double f = 1 / 298.257223563; // achatamento da Terra

  double L = (lon2 - lon1) * M_PI / 180;
  double U1 = atan((1 - f) * tan(lat1 * M_PI / 180));
  double U2 = atan((1 - f) * tan(lat2 * M_PI / 180));
  double sinU1 = sin(U1), cosU1 = cos(U1);
  double sinU2 = sin(U2), cosU2 = cos(U2);

  double lambda = L, lambdaP, sinLambda, cosLambda, sinSigma, cosSigma, sigma, sinAlpha, cosSqAlpha, cos2SigmaM, C;
  int iterLimit = 100;
  do
  {
    sinLambda = sin(lambda), cosLambda = cos(lambda);
    sinSigma = sqrt((cosU2 * sinLambda) * (cosU2 * sinLambda) +
                    (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda));
    if (sinSigma == 0)
      return 0; // pontos coincidentes
    cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
    sigma = atan2(sinSigma, cosSigma);
    sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
    cosSqAlpha = 1 - sinAlpha * sinAlpha;
    cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;
    C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
    lambdaP = lambda;
    lambda = L + (1 - C) * f * sinAlpha *
                     (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
  } while (fabs(lambda - lambdaP) > 1e-12 && --iterLimit > 0);

  if (iterLimit == 0)
    return 0; // fórmula não convergiu

  double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
  double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
  double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
  double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
  double s = b * A * (sigma - deltaSigma);

  return s;
}

// Verifica se o dispositivo ultrapassou o limite de distância
bool deviceIsTooFar(float lat, float lon, String *distance)
{
  double dist = vincenty(lat, lon, LATITUDE_REFERENCE, LONGITUDE_REFERENCE);

  *distance = String(dist);

  if (dist > MAX_DISTANCE)
    return true;

  return false;
}

// será chamada sempre que houver uma mensagem MQTT recebida, conexão perdida ou estabelecida, etc.
void callback(char *topic, byte *payload, unsigned int length)
{

  // dados recebidos do tracker
  deserializeJson(doc, payload);
  const char *sensor = doc["sensor"];
  const char *device = doc["device"];
  float latitude = doc["coordinates"]["lat"];
  float longitude = doc["coordinates"]["lon"];

  String distance;

  if (deviceIsTooFar(latitude, longitude, &distance))
  {

    if (client.connected())
    {
      String client_id = "esp32-";
      client_id += String(WiFi.macAddress());
      String topic = String(MQTT_TOPIC_PREFIX) + "/" + client_id;
      Serial.println("...................Device is far....................");
      String new_data;
      doc2.clear();
      doc2["device"] = "esp32";
      doc2["sensor"] = client_id;
      doc2["distance"] = String(distance);
      doc2["isClose"] = false;
      doc2["referenceLocation"]["lat"] = String(LATITUDE_REFERENCE, 6);
      doc2["referenceLocation"]["lon"] = String(LONGITUDE_REFERENCE, 6);
      doc2["referenceLocation"]["max_distance"] = String(MAX_DISTANCE);
      serializeJson(doc2, new_data);
      client.publish(topic.c_str(), new_data.c_str());
      Serial.println(new_data);
      Serial.println(topic);
      Serial.println("...................................................");
    }

    digitalWrite(LED_EXT, LOW); // apaga o led sinalizando que o dispositivo está muito longe
    Serial.println("{Lat: " + String(latitude, 6) + "," + " Long: " + String(longitude, 6) + "," + distance + "m }");
  }
  else
  {

    if (client.connected())
    {
      String client_id = "esp32-";
      client_id += String(WiFi.macAddress());
      String topic = String(MQTT_TOPIC_PREFIX) + "/" + client_id;
      Serial.println("...................Device is close....................");
      String new_data;
      doc2.clear();
      doc2["device"] = "esp32";
      doc2["sensor"] = client_id;
      doc2["distance"] = String(distance);
      doc2["isClose"] = true;
      doc2["referenceLocation"]["lat"] = String(LATITUDE_REFERENCE, 6);
      doc2["referenceLocation"]["lon"] = String(LONGITUDE_REFERENCE, 6);
      doc2["referenceLocation"]["max_distance"] = String(MAX_DISTANCE);
      serializeJson(doc2, new_data);
      client.publish(topic.c_str(), new_data.c_str());
      Serial.println(new_data);
      Serial.println(topic);
      Serial.println("...................................................");
    }

    digitalWrite(LED_EXT, HIGH); // acende led sinalizando que o dispositivo está muito perto
    Serial.println("{Lat: " + String(latitude, 6) + "," + " Long: " + String(longitude, 6) + "," + distance + "m }");
  }
}

// Função de inicialização, após conectar ao wifi inicializa o servidor
void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(LED_EXT, OUTPUT);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  connectToMqtt();
}

// Função de loop
void loop()
{
  // Mantenha a conexão MQTT ativa
  if (!client.connected())
  {
    connectToMqtt();
  }
  client.loop();
}