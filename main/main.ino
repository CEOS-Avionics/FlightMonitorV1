/**** Inclusões ****/ 
// Vou tentar fazer sem Wire.h e SPI.h porque acho que elas já são inclusas pelas libs dos sensores, SD, GPS... 
#include "WiFi.h"
#include "WiFiUdp.h"
#include "SdFat.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP3XX.h"
#include "TinyGPSPlus.h"
#include "ICM20948_WE.h" // lib feita pela SparkFun.
#include "EEPROM.h"
#undef F // É preciso undefinir F, porque ele é a macro da memória flash, mas também é o símbolo usado para a jacobiana F.
#include "C:\Users\henri\OneDrive\Documentos\KalmanFilter\kalman-master\include\kalman\ExtendedKalmanFilter.hpp"
#include "C:\Users\henri\OneDrive\Documentos\Codigos_do_controlador_V1\KalmanABSOLUTO\KalmanModeling\SystemModel.hpp"
#include "C:\Users\henri\OneDrive\Documentos\Codigos_do_controlador_V1\KalmanABSOLUTO\KalmanModeling\bmpMeasurementModel.hpp"
#include "C:\Users\henri\OneDrive\Documentos\Codigos_do_controlador_V1\KalmanABSOLUTO\KalmanModeling\gpsMeasurementModel.hpp"
#include "C:\Users\henri\OneDrive\Documentos\Codigos_do_controlador_V1\KalmanABSOLUTO\KalmanModeling\icmMeasurementModel.hpp"

/**** Declarações ****/
// Objetos
WiFiUDP udp;
SdExFat sd;
ExFile f;
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
ICM20948_WE icm;
// struct para guardar os bias acessados da EEPROM.
struct biasStore {
  int32_t header;
  int32_t biasGyroX;
  int32_t biasGyroY;
  int32_t biasGyroZ;
  int32_t biasAccelX;
  int32_t biasAccelY;
  int32_t biasAccelZ;
  int32_t biasCPassX;
  int32_t biasCPassY;
  int32_t biasCPassZ;
  int32_t sum;
} store;
// Declarações de modelos e vetores do filtro de Kalman.
SystemModel<float> sys;                  State<float> x; Control<float> u;
bmpMeasurementModel<float> bmpModel;     bmpMeasurement<float> bmpMeasure;
gpsMeasurementModel<float> gpsModel;     gpsMeasurement<float> gpsMeasure;
icmMeasurementModel<float> icmModel;     icmMeasurement<float> icmMeasure;
// Matrizes de covariância dos ruídos de processo (Q) e de medida (R).
Kalman::Covariance<Kalman::Vector<float, 28>> Q;
Kalman::Covariance<Kalman::Vector<float, 1>> Rbmp;
Kalman::Covariance<Kalman::Vector<float, 5>> Rgps;
Kalman::Covariance<Kalman::Vector<float, 9>> Ricm;
// O filtro em si.
Kalman::ExtendedKalmanFilter<State<float>> ekf;
// Referencias de altitude (para o bmp) e posição (para o gps).
float refAltitude;
float refPX, refPY, refPZ;
Eigen::Quaternion<float> initialQuaternion;
// Variáveis de tempo, contagem e flags.
float last_instant = 0;
float actual_instant = 0;
int counter = 0; // conta o número de iterações do loop 
char no_local_de_lancamento = 'n';
char voando = 'n';
bool drogueDeployed = false;

/**** Definições ****/
// Especificações da comunicação WiFi via UDP.
const char* ssid = "ALHN-9DA1";
const char* password = "4671109028";
const char* udpAddress = "192.168.1.64";  
const int udpPort = 12345;  
// Definições do referencial absoluto;
Eigen::Vector3f gPrime{0.0f, 0.0f, 1.0f}/*eixo Zabsoluto*/, hPrime{0.0f, 1.0f, 0.0f}/*eixo Yabsoluto*/;
// Definições relativas a seensores.
#define BMP_I2C_ADDRESS (0x76)
#define FALL_ACCELERATION (-1234567890)
#define MAIN_DEPLOYMENT_ALTITUDE (1234567890)
#define PARACHUTE_EN_PIN (11)
#define MAIN_PIN (8)
#define DROGUE_PIN (12)
#define ICM_I2C_ADDRESS (0x68) 
ICM20948_accRange currentAccRange = ICM20948_ACC_RANGE_2G;
ICM20948_gyroRange currentGyrRange = ICM20948_GYRO_RANGE_250;
#define SD_CS_PIN (5)
#define SPI_SPEED SD_SCK_MHZ(4)
#define SEALEVELPRESSURE (1013.25)
const char* filename = "kalman.csv";
// configurar, para cada sensor, o número de ciclos passados até ele realizar uma medida.
const int Nbmp = 5;
const int Ngps = 10;
const int Nicm = 1;

/**** Protótipos ****/
void getReferenceAltitude();
void getReferencePosition();
void checkAndAdjustAccelRange(float x, float y, float z);
void checkAndAdjustGyroRange(float x, float y, float z);
xyzFloat getAccValuesInMs2();
xyzFloat getGyrValuesInDpS();
Eigen::Vector3f getMagneticFieldOnSite(); // Coleta o campo magnético inicial a ser atribuído ao vetor de estados.
Eigen::Vector3f getOrtogonalMagneticFieldOnSite(Eigen::Vector3f h, const Eigen::Vector3f& g); // coleta o campo magnético que aponta puramente para o norte.
Eigen::Vector3f getGravityOnSite(); 
// Essa função é responsável por achar a orientação do foguete no local de lançamento, esta orientação é 
// relaciona o referencial do icm20948 (XYZ) com o referencial absoluto (NED) a partir da medição da gravidade (g)   
// e do campo magnético (h) no local de lançamento. hPrime e gPrime são os vetores gravidade e campo magnético no 
// referencial absoluto (na verdade o referencial absoluto é definido a partir deles) gPrime = ||g||*k e hPrime = ||h||*j.
// Os componentes do quartenion inicial são guardadas em q.
bool findRotationQuaternion(const Eigen::Vector3f& g, const Eigen::Vector3f& h, 
                           const Eigen::Vector3f& gPrime, const Eigen::Vector3f& hPrime, 
                           Eigen::Quaternion<float>& q);
void writeOnSD();
bool deployMain();
bool deployDrogue();

void setup() {

  // Inicia comunicação UDP para envio recebimento de flags e envio de mensagens.
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) delay(1000);
  udp.begin(udpPort);

  // Inicializa barramento I2C.
  Wire.begin();
  Wire.setClock(400000);

  // Inicializa e configura o BMP388.
  while(!bmp.begin_I2C(BMP_I2C_ADDRESS))
  {
    udp.beginPacket(udpAddress, udpPort);
    udp.print("Falha ao iniciar BMP.\n");
    udp.endPacket();
    delay(500);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp.performReading(); // Realiza a primeira medida para retirar o valor 1895.40 quem vem como lixo

  // Inicializa o barramento serial a ser usado pelo gps.
  Serial.begin(115200); 
  while(!Serial);

  // Inicializa e configura o ICM20948.
  icm = ICM20948_WE(ICM_I2C_ADDRESS);
  while (!icm.init())
  {
    udp.beginPacket(udpAddress, udpPort);
    udp.print("Falha ao inicializar icm.\n");
    udp.endPacket();
    delay(500);
  }
  while (!icm.initMagnetometer())
  {
    udp.beginPacket(udpAddress, udpPort);
    udp.print("Falha ao inicializar magnetometro.\n");
    udp.endPacket();
    delay(500);
  }
  icm.setAccRange(currentAccRange);
  icm.setGyrRange(currentGyrRange);

  // Inicializa o cattão SD e apaga algum arquivo anterior que tenha o mesmo nomme.
  while(!sd.begin(SD_CS_PIN, SPI_SPEED))
  {
    udp.beginPacket(udpAddress, udpPort);
    udp.print("Falha ao inicializar cartao SD.\n");
    udp.endPacket();
    delay(500);
  }
  if (sd.exists(filename)) sd.remove(filename);

  // Matriz de covariância de ruído de processo.
  Q.setZero(); // não é esperado que biases e campo magnético sofram mudança alguma.
  Q.diagonal() << 
    0.25, 0.25, 0.25, 0.25,  // quaternion
    60, 60, 60,              // velocidade angular
    0.25, 0.25, 0.25, 0.25, 0.25, 0.25,  // posição e velocidade
    100, 100, 100;           // aceleração
  sys.setCovariance(Q);

  // Matrizes de covariância de ruído de medida.
  Rbmp.diagonal() <<
    0.12; // altitude (m)
  bmpModel.setCovariance(Rbmp);

  Rgps.setZero();
  Rgps.diagonal() <<
    2.12, 2.12, 3.20, // posições (m)
    0.1, // velocidade (m/s)
    0.5; // erro de orientação (degree)
  gpsModel.setCovariance(Rgps);

  // O erro de medida do icm depende da taxa de amostragem, aqui são os erros para taxa de 100 Hz.
  Ricm.setZero();
  Ricm.diagonal() <<
    2300 * 1e-6, 2300 * 1e-6, 2300 * 1e-6, // acelerômetro (g)
    0.15, 0.15, 0.15, // giroscópio (dps)
    0.15, 0.15, 0.15; // magnetômetro (micro T), este não depende da taxa de amostragem.
  icmModel.setCovariance(Ricm);

  // Pega os biases do ICM20948 que estão guardados na EEPROM da ESP32 e guardao-os em store.
  EEPROM.begin(128);
  EEPROM.get(0, store);
  // Converte biases dos valores brutos para as unidades: m/s^2, dps, microT.
  store.biasGyroX /= 131.0f; 
  store.biasGyroY /= 131.0f; 
  store.biasGyroZ /= 131.0f; 
  store.biasAccelX *= (9.81f / 16384.0f); 
  store.biasAccelY *= (9.81f / 16384.0f); 
  store.biasAccelZ *= (9.81f / 16384.0f); 
  store.biasCPassX *= 0.15f; 
  store.biasCPassY *= 0.15f; 
  store.biasCPassZ *= 0.15f;

  // Espera até que o foguete esteja no local de lançamento para coletar a altitude e posição de referência.
  while(no_local_de_lancamento != 'y')
  {
    if(udp.parsePacket()){
      char packetBuffer[99]; // Lê um buffer com até 99 caracteres e pega só o primeiro como flag.
      no_local_de_lancamento = udp.read(packetBuffer, 1);
    } 
    delay(500);
  }

  getReferenceAltitude(); // variável global refAltitude foi atualizada.
  getReferencePosition(); // variável globais refPX, refPY, refPZ foram atualizadas.

  x.setZero(); // Foguete se encontra em repouso e na posição (0,0,0).

  // gravidade no referencial XYZ do sensor. Altera o módulo da variável global gPrime.
  Eigen::Vector3f GravityOnSite = getGravityOnSite();
  // campo magnético no referencial XYZ do sensor. 
  Eigen::Vector3f MagFieldOnSite = getMagneticFieldOnSite();
  // retorna um campo magnético ortogonal à grvidade em XYZ. Altera o módulo da variável global hPrime.
  Eigen::Vector3f MagOrtogonalToGrav = getOrtogonalMagneticFieldOnSite(MagFieldOnSite, GravityOnSite); 
  // Encontra a orientação inicial do foguete.
  findRotationQuaternion (GravityOnSite, MagOrtogonalToGrav, gPrime, hPrime, initialQuaternion);

  /* Atribuição de valores ao vetor de estados */
  // Campo magnético no referencial NED. É necessário rotacioná-lo.
  // O operador * faz a operação qmq* na lib Eigen quando se multiplica por um quaternion. 
  MagFieldOnSite = initialQuaternion*MagFieldOnSite;
  x.mx() = MagFieldOnSite(0);
  x.my() = MagFieldOnSite(1);
  x.mz() = MagFieldOnSite(2);
  // Quaternion (orientação inicial do foguete). 
  // A rotação do referencial do foguete foi oposta à rotação dos vetores usada logo acima.
  initialQuaternion = initialQuaternion.conjugate();
  x.q0() = initialQuaternion.w();
  x.q1() = initialQuaternion.x();
  x.q2() = initialQuaternion.y();
  x.q3() = initialQuaternion.z();
  // Biases do icm.
  x.gbx() = store.biasGyroX; x.gby() = store.biasGyroY; x.gbz() = store.biasGyroZ;
  x.abx() = store.biasAccelX; x.aby() = store.biasAccelY; x.abz() = store.biasAccelZ;
  x.mbx() = store.biasCPassX; x.mby() = store.biasCPassY; x.mbz() = store.biasCPassZ;

  // Inicializa o filtro de Kalman.
  ekf.init(x);

  // A primeira linha do arquivo armazena os biases.
  f = sd.open(filename, FILE_WRITE);
  f.print(store.biasAccelX); f.print(",");
  f.print(store.biasAccelY); f.print(",");
  f.print(store.biasAccelZ); f.print(",");
  f.print(store.biasGyroX); f.print(",");
  f.print(store.biasGyroY); f.print(",");
  f.print(store.biasGyroZ); f.print(",");
  f.print(store.biasCPassX); f.print(",");
  f.print(store.biasCPassY); f.print(",");
  f.println(store.biasCPassZ); 

  // Envia a mensagem de que tudo foi inicializado, dando LUZ VERDE para o lançamento, basta enviar a flag.
  udp.beginPacket(udpAddress, udpPort);
  udp.print("Foguete pronto para o lançamento!\n");
  udp.endPacket();

  // Espera até que o lançamento ocorra para prosseguir com as medições e o filtro.
  while(voando != 'y')
  {
    if(udp.parsePacket()){
      char packetBuffer[99]; // Lê um buffer com até 99 caracteres e pega só o primeiro como flag.
      no_local_de_lancamento = udp.read(packetBuffer, 1);
    } 
    delay(500);
  }

  last_instant = micros(); // Solução rápida para que o dt não fique gigante na primeira iteração do loop.
}

void loop() {
  actual_instant = micros();
  auto dt = (actual_instant - last_instant) * 1e-6; //dt em segundos.
  
  sys.dt = dt;
  
  ekf.predict(sys,u);

  // Update de medida do bmp.
  if(counter % Nbmp == 0)
  {
    bmp.performReading();
    bmpMeasure.pz() = bmp.readAltitude(refAltitude);
    ekf.update(bmpModel, bmpMeasure);
  }

  // Update de medida do gps. Ele não funciona bem com acelerações acima de 4g.
  if(counter % Ngps == 0 && x.az() < 40)
  {
    bool location = false;
    bool altitude = false;
    bool course = false;
    bool speed = false;

    while(Serial.available()){
      gps.encode(Serial.read());

      // Verifica se os dados já foram lidos por completo e se são dados novos.
      if(gps.location.isUpdated() && gps.location.isValid()) location = true;
      if(gps.altitude.isUpdated() && gps.altitude.isValid()) altitude = true;
      if(gps.course.isUpdated() && gps.course.isValid()) course = true;
      if(gps.speed.isUpdated() && gps.speed.isValid()) speed = true; 
      
      // Encerra a leitura de dados. 
      if(location && altitude && course && speed) break;
    }

    gpsMeasure.px() = (gps.location.lng() - refPX) * 111111; // 1 grau de latitude é aproximadamente 111.111 m.
    gpsMeasure.py() = (gps.location.lat() - refPY) * 111111;
    gpsMeasure.pz() = gps.altitude.meters() - refPZ;
    gpsMeasure.v() = gps.speed.mps();
    gpsMeasure.theta() = gps.course.deg();
    ekf.update(gpsModel, gpsMeasure);
  }

  // Update de medida do icm.
  if(counter % Nicm == 0)
  {
    icm.readSensor();
    
    xyzFloat accMs2; // Aceleração não corrigida (com bias) em metros por segundo ao quadrado.
    xyzFloat gyrDpS; // Velocidade angular não corrigida (com bias) em graus por segundo.
    xyzFloat magValue; // Campo magnético em microTesla.

    accMs2 = getAccValuesInMs2();
    gyrDpS = getGyrValuesInDpS();
    icm.getMagValues(&magValue);

    icmMeasure.ax() = accMs2.x;
    icmMeasure.ay() = accMs2.y;
    icmMeasure.az() = accMs2.z;
    icmMeasure.wx() = gyrDpS.x;
    icmMeasure.wy() = gyrDpS.y;
    icmMeasure.wz() = gyrDpS.z;
    icmMeasure.mx() = magValue.x;
    icmMeasure.my() = magValue.y;
    icmMeasure.mz() = magValue.z;
    ekf.update(icmModel, icmMeasure);

    // Verifica se é necessário ajustar a sensibilidade do sensor.
    checkAndAdjustAccelRange(x.ax(), x.ay(), x.az());
    checkAndAdjustGyroRange(x.wx(), x.wy(), x.wz());
  }
  
  // Verifica as condições para liberação dos para-quedas.
  if(x.az() < FALL_ACCELERATION) 
  {
    drogueDeployed = deployDrogue();
  }
  if(drogueDeployed && x.pz() < MAIN_DEPLOYMENT_ALTITUDE)
  {
    deployMain();
  }

  writeOnSD();

  last_instant = actual_instant;
  counter++;
}

void getReferenceAltitude()
{
  float soma = 0;
  for(int i=0; i<32; i++){
    bmp.performReading();
    soma += bmp.readAltitude(1013.25);
  }
  refAltitude = soma/32;
}

void getReferencePosition()
{
  // GPS envia string de dados pelo barramento serial, a lib tinyGPS++ é responsável por codificar essa string e exibí-la.
  bool location = false;
  bool altitude = false;

  while(Serial.available()){
    gps.encode(Serial.read());

    // Verifica se os dados já foram lidos por completo e se são dados novos.
    if(gps.location.isUpdated() && gps.location.isValid()) location = true;
    if(gps.altitude.isUpdated() && gps.altitude.isValid()) altitude = true;
   
    // Encerra a leitura de dados. 
    if(location && altitude) break;
  }

  float soma1 = 0;
  float soma2 = 0;
  float soma3 = 0;
  for(int i=0; i<5; i++)
  {
    soma1 += gps.location.lng();
    soma2 += gps.location.lat();
    soma3 += gps.altitude.meters();
    delay(1000);
  }
  refPX = soma1/5;
  refPY = soma2/5;
  refPZ = soma3/5; 
}

void writeOnSD()
{
  // Instante atual na primeira linha, elementos do vetor de estado na segunda e terceira linhas
  // e medidas brutas na quarta. Dividi os elementos do vetor de estados para  caberem no buffer de 
  // leitura da função readString(), usada no código lerSD.ino 
  f.print(actual_instant); f.println();
  // Vetor de estados.
  f.print(x.q0()); f.print(",");
  f.print(x.q1()); f.print(",");
  f.print(x.q2()); f.print(",");
  f.print(x.q3()); f.print(",");
  f.print(x.wx()); f.print(",");
  f.print(x.wy()); f.print(",");
  f.print(x.wz()); f.print(",");
  f.print(x.px()); f.print(",");
  f.print(x.py()); f.print(",");
  f.print(x.pz()); f.print(",");
  f.print(x.vx()); f.print(",");
  f.print(x.vy()); f.print(",");
  f.print(x.vz()); f.print(",");
  f.print(x.ax()); f.print(",");
  f.print(x.ay()); f.print(",");
  f.print(x.az()); f.println();
  f.print(x.abx()); f.print(",");
  f.print(x.aby()); f.print(",");
  f.print(x.abz()); f.print(",");
  f.print(x.gbx()); f.print(",");
  f.print(x.gby()); f.print(",");
  f.print(x.gbz()); f.print(",");
  f.print(x.mx()); f.print(",");
  f.print(x.my()); f.print(",");
  f.print(x.mz()); f.print(",");
  f.print(x.mbx()); f.print(",");
  f.print(x.mby()); f.print(",");
  f.print(x.mbz()); f.println();
  // bmpMeasure
  f.print(bmpMeasure.pz()); f.print(",");
  // gpsMeasure
  f.print(gpsMeasure.px()); f.print(",");
  f.print(gpsMeasure.py()); f.print(",");
  f.print(gpsMeasure.pz()); f.print(",");
  f.print(gpsMeasure.v()); f.print(",");
  f.print(gpsMeasure.theta()); f.print(",");
  // icmMeasure
  f.print(icmMeasure.ax()); f.print(",");
  f.print(icmMeasure.ay()); f.print(",");
  f.print(icmMeasure.az()); f.print(",");
  f.print(icmMeasure.wx()); f.print(",");
  f.print(icmMeasure.wy()); f.print(",");
  f.print(icmMeasure.wz()); f.print(",");
  f.print(icmMeasure.mx()); f.print(",");
  f.print(icmMeasure.my()); f.print(",");
  f.print(icmMeasure.mz()); f.println();
}

// Verifica saturação e limite inferior do acelerômetro
void checkAndAdjustAccelRange(float x, float y, float z) 
{
  float maxRange, prevRange;
  switch (currentAccRange) {
    case ICM20948_ACC_RANGE_2G: maxRange = 2.0; prevRange = 0.0; break; // Não há range menor que 2g
    case ICM20948_ACC_RANGE_4G: maxRange = 4.0; prevRange = 2.0; break;
    case ICM20948_ACC_RANGE_8G: maxRange = 8.0; prevRange = 4.0; break;
    case ICM20948_ACC_RANGE_16G: maxRange = 16.0; prevRange = 8.0; break;
  }

  // Thresholds: 90% do range atual (superior) e 90% do range anterior (inferior)
  float upperThreshold = maxRange * 0.9;
  float lowerThreshold = prevRange * 0.9;

  float maxVal = max(max(abs(x), abs(y)), abs(z));

  if (maxVal > upperThreshold) {
    if (currentAccRange < ICM20948_ACC_RANGE_16G) { // Aumenta se não estiver no máximo
      icm.setAccRange(static_cast<ICM20948_accRange>(currentAccRange + 1));
    }
  } else if (maxVal < lowerThreshold && currentAccRange > ICM20948_ACC_RANGE_2G) { // Diminui se não estiver no mínimo
    icm.setAccRange(static_cast<ICM20948_accRange>(currentAccRange - 1));
  }
}

// Verifica saturação e limite inferior do giroscópio
void checkAndAdjustGyroRange(float x, float y, float z) 
{
  float maxRange, prevRange;
  switch (currentGyrRange) {
    case ICM20948_GYRO_RANGE_250: maxRange = 250.0; prevRange = 0.0; break; // Não há range menor que 250dps
    case ICM20948_GYRO_RANGE_500: maxRange = 500.0; prevRange = 250.0; break;
    case ICM20948_GYRO_RANGE_1000: maxRange = 1000.0; prevRange = 500.0; break;
    case ICM20948_GYRO_RANGE_2000: maxRange = 2000.0; prevRange = 1000.0; break;
  }

  // Thresholds: 90% do range atual (superior) e 90% do range anterior (inferior)
  float upperThreshold = maxRange * 0.9;
  float lowerThreshold = prevRange * 0.9;

  float maxVal = max(max(abs(x), abs(y)), abs(z));

  if (maxVal > upperThreshold) {
    if (currentGyrRange < ICM20948_GYRO_RANGE_2000) { // Aumenta se não estiver no máximo
      icm.setGyrRange(static_cast<ICM20948_gyroRange>(currentGyrRange + 1));
    }
  } else if (maxVal < lowerThreshold && currentGyrRange > ICM20948_GYRO_RANGE_250) { // Diminui se não estiver no mínimo
    icm.setGyrRange(static_cast<ICM20948_gyroRange>(currentGyrRange - 1));
  }
}

xyzFloat getAccValuesInMs2() 
{
  xyzFloat accRaw; icm.getAccRawValues(&accRaw); // Valores brutos
  xyzFloat acc;
  float sensitivity;

  switch (currentAccRange) {
    case ICM20948_ACC_RANGE_2G:  sensitivity = 16384.0 / 9.81; break; // ≈ 160667.04 LSB/(m/s²)
    case ICM20948_ACC_RANGE_4G:  sensitivity = 8192.0 / 9.81;  break; // ≈ 80333.52 LSB/(m/s²)
    case ICM20948_ACC_RANGE_8G:  sensitivity = 4096.0 / 9.81;  break; // ≈ 40166.76 LSB/(m/s²)
    case ICM20948_ACC_RANGE_16G: sensitivity = 2048.0 / 9.81;  break; // ≈ 20083.38 LSB/(m/s²)
    default: sensitivity = 16384.0 / 9.81; // Padrão
  }

  acc.x = accRaw.x / sensitivity;
  acc.y = accRaw.y / sensitivity;
  acc.z = accRaw.z / sensitivity;

  return acc;
}

xyzFloat getGyrValuesInDpS() 
{
  xyzFloat gyrRaw; icm.getGyrRawValues(&gyrRaw); // Valores brutos
  xyzFloat gyr;
  float sensitivity;

  switch (currentGyrRange) {
    case ICM20948_GYRO_RANGE_250:  sensitivity = 131.0; break;     
    case ICM20948_GYRO_RANGE_500:  sensitivity = 65.5;  break; 
    case ICM20948_GYRO_RANGE_1000: sensitivity = 32.8;  break; 
    case ICM20948_GYRO_RANGE_2000: sensitivity = 16.4;  break; 
    default: sensitivity = 131.0; // Padrão
  }

  gyr.x = gyrRaw.x / sensitivity;
  gyr.y = gyrRaw.y / sensitivity;
  gyr.z = gyrRaw.z / sensitivity;

  return gyr;
}

Eigen::Vector3f getMagneticFieldOnSite()
{
  Eigen::Vector3f mos;
  float somax = 0;
  float somay = 0;
  float somaz = 0;
  
  for(int i=0; i<10; i++)
  {
    icm.readSensor();
    xyzFloat MagValue;
    icm.getMagValues(&MagValue);
    somax += MagValue.x;
    somay += MagValue.y;
    somaz += MagValue.z;
    delay(100);
  } 

  somax /= 10;
  somay /= 10;
  somaz /= 10;
  // descontar os biases
  somax -= store.biasCPassX;
  somay -= store.biasCPassY;
  somaz -= store.biasCPassZ;

  mos(0) = somax; mos(1) = somay; mos(2) = somaz;
  return mos;
}

Eigen::Vector3f getGravityOnSite()
{
  Eigen::Vector3f gos;
  float somax = 0;
  float somay = 0;
  float somaz = 0;

  for(int i=0; i<10; i++)
  {
    xyzFloat gravity;
    icm.readSensor();
    gravity = getAccValuesInMs2();
    somax += gravity.x;
    somay += gravity.y;
    somaz += gravity.z;
    delay(100);
  }

  somax /= 10;
  somay /= 10;
  somaz /= 10;
  // descontar os biases
  somax -= store.biasAccelX;
  somay -= store.biasAccelY;
  somaz -= store.biasAccelZ;
  
  gos(0) = somax; gos(1) = somay; gos(2) = somaz;
  // Altera a variável global gPrime para que tenha a mesma norma que GravityOnSite.
  // Isso é necessário para a função findRotationQuaternion.
  gPrime *= gos.norm(); 

  return gos;
}

Eigen::Vector3f getOrtogonalMagneticFieldOnSite(Eigen::Vector3f h, const Eigen::Vector3f& g)
{
  Eigen::Vector3f ortogonal;
  Eigen::Vector3f projecao_h_em_g = (h.dot(g)/g.squaredNorm())*g;

  ortogonal = h - projecao_h_em_g;
  // Altera variável global hPrime para uso correto da função findRotationQuaternion,
  // ela exige que o vetor original e o rotacionado tenham mesmo módulo.
  hPrime *= ortogonal.norm();

  return ortogonal;
}

bool findRotationQuaternion(const Eigen::Vector3f& g, const Eigen::Vector3f& h, 
                           const Eigen::Vector3f& gPrime, const Eigen::Vector3f& hPrime, 
                           Eigen::Quaternion<float>& q) {
    // Passo 1: Verificar normas
    float normGPrime = gPrime.norm();
    float normHPrime = hPrime.norm();
    float normG = g.norm();
    float normH = h.norm();

    const float epsilon = 1e-6; // Tolerância ajustada para o tipo T
    if (std::abs(normGPrime - normG) > epsilon || std::abs(normHPrime - normH) > epsilon) {
        udp.beginPacket(udpAddress, udpPort);
        udp.print("Normas não são iguais. Não há solução.\n");
        udp.endPacket();
        return false;
    }
    if (normGPrime < epsilon || normHPrime < epsilon) {
        udp.beginPacket(udpAddress, udpPort);
        udp.print("Vetor nulo detectado. Não há solução.\n");
        udp.endPacket();
        return false;
    }

    // Passo 2: Verificar ângulos relativos
    float cosThetaPrime = gPrime.dot(hPrime) / (normGPrime * normHPrime);
    float cosTheta = g.dot(h) / (normG * normH);
    if (std::abs(cosThetaPrime - cosTheta) > epsilon) {
        udp.beginPacket(udpAddress, udpPort);
        udp.print("Ângulos relativos não são preservados. Não há solução.\n");
        udp.endPacket();
        return false;
    }

    // Passo 3: Construir base ortonormal inicial e final
    Eigen::Vector3f uPrime = gPrime.normalized();
    Eigen::Vector3f wPrime = gPrime.cross(hPrime).normalized();
    Eigen::Vector3f vPrime = wPrime.cross(uPrime);

    Eigen::Vector3f u = g.normalized();
    Eigen::Vector3f w = g.cross(h).normalized();
    Eigen::Vector3f v = w.cross(u);

    // Passo 4: Construir matriz de rotação R
    Eigen::Matrix3f basisPrime, basisFinal;
    basisPrime.col(0) = uPrime;
    basisPrime.col(1) = hPrime.normalized();
    basisFinal.col(2) = wPrime;

    basisFinal.col(0) = u;
    basisFinal.col(1) = h.normalized();
    basisFinal.col(2) = w;

    Eigen::Matrix3f R = basisFinal * basisPrime.transpose();

    // Passo 5: Converter para quaternion
    q = Eigen::Quaternion<float>(R);

    // Passo 6: Verificar
    Eigen::Vector3f gRotated = q * gPrime;
    Eigen::Vector3f hRotated = q * hPrime;
    if ((gRotated - g).norm() > epsilon || (hRotated - h).norm() > epsilon) {
        udp.beginPacket(udpAddress, udpPort);
        udp.print("Erro: Quaternion não rotaciona corretamente os vetores.\n");
        udp.endPacket();
        return false;
    }

    return true;
}

bool deployMain()
{
  digitalWrite(PARACHUTE_EN_PIN, HIGH);
  digitalWrite(MAIN_PIN, HIGH);
  delay(100);
  digitalWrite(PARACHUTE_EN_PIN, LOW);
  return true;
}

bool deployDrogue()
{
  digitalWrite(PARACHUTE_EN_PIN, HIGH);
  digitalWrite(DROGUE_PIN, HIGH);
  delay(100);
  digitalWrite(PARACHUTE_EN_PIN, LOW);
  return true;
}