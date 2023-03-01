#include <WiFi.h>
#include <PubSubClient.h>
// #include <SoftwareSerial.h>
#include <cmath>
#include <ArduinoJson.h>

#define RXD2 16                            // RXD2 do ESP32 conectado ao TX do GPS
#define TXD2 17                            // TXD2 do ESP32 conectado ao RX do GPS
#define GPS_BAUDRATE 9600                  // Baud rate do GPS
#define WIFI_SSID "PRUU"                   // SSID da rede WiFi
#define WIFI_PASSWORD "gustavoejulia"      // Senha da rede WiFi
#define MQTT_SERVER "broker.emqx.io"       // Endereço do servidor MQTT
#define MQTT_PORT 1883                     // Porta do servidor MQTT
#define MQTT_TOPIC "esp32/gps/coordinates" // Tópico MQTT para publicar as coordenadas
#define MQTT_USERNAME "emqx"               // Usuario do servidor
#define MQTT_PASSWORD "public"             // Senha do usuario
#define LED 2                              // Led interno do esp32
#define LED_EXT 15                         // Led externo ao esp32

const double LATITUDE_REFERENCE = -22.129688;  // Latitude de referência para calcular o distanciamento do dispositivo
const double LONGITUDE_REFERENCE = -51.408570; // Longitude de referência para calcular o distanciamento do dispositivo
const double MAX_DISTANCE = 1000;              // Distância limite usada para disparar SMS de alerta de distanciamento, medida em metros

// SoftwareSerial ss(RXD2, TXD2);        // Define a serial para comunicação com o GPS
WiFiClient wifiClient;                // Cliente WiFi
PubSubClient client(wifiClient);      // Cliente MQTT
const int json = JSON_OBJECT_SIZE(6); // Tamanho do objeto
StaticJsonDocument<json> doc;         // Criação do documento (objeto que vai ser publicado)

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

void callback(char *topic, byte *payload, unsigned int length)
{
  deserializeJson(doc, payload);

  const char *sensor = doc["sensor"];
  const char *device = doc["device"];
  float latitude = doc["coordinates"]["lat"];
  float longitude = doc["coordinates"]["lon"];

  String distance;

  if (deviceIsTooFar(latitude, longitude, &distance))
  {

    Serial.println("Device is too far");
    digitalWrite(LED_EXT, LOW); // apaga o led sinalizando que o dispositivo está muito longe
    Serial.println("{Lat: " + String(latitude, 6) + "," + " Long: " + String(longitude, 6) + "," + distance + "m }");
  }
  else
  {
    Serial.println("Device is too close");
    digitalWrite(LED_EXT, HIGH); // acende led sinalizando que o dispositivo está muito perto
    Serial.println("{Lat: " + String(latitude, 6) + "," + " Long: " + String(longitude, 6) + "," + distance + "m }");
  }
  delay(5000);
}
// Função de inicialização, após conectar ao wifi inicializa o servidor
void setup()
{
  Serial.begin(115200);
  // ss.begin(GPS_BAUDRATE);
  pinMode(LED, OUTPUT);
  pinMode(LED_EXT, OUTPUT);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  if (client.connect("client_esp32"))
  {
    // connection succeeded
    Serial.println("Connected");
    boolean r = client.subscribe("esp32/gps/coordinates");
    Serial.println("Subscribed");
  }
  else
  {
    Serial.println("Connection failed ");
  }
}

// Função de loop
void loop()
{
  client.loop();
}