#include <SPI.h>
#include <SdFat.h>

#define CS 5
#define SPI_SPEED SD_SCK_MHZ(4)
const char* filename = "kalman.csv";

SdExFat sd;
ExFile f;
String instant;
String state_vector1;
String state_vector2;
String measurements_with_bias;

char flag = 'w';

void setup() {
  Serial.begin(9600);
  while(!Serial);

  if(!sd.begin(CS, SPI_SPEED)){
    Serial.println("Falha ao inicializar cartao SD.");
    return;
  }

  if (sd.exists(filename)) 
    f = sd.open(filename); // Abre o arquivo no formato de leitura por default.
  else
    Serial.println("Nar ha arquivo com este nome no cartao!");

  // Espera até ser enviado o sinal para leitura do arquivo.
  while(flag != 'r'){
    if(Serial.available()){
      flag = Serial.read();
      while(Serial.available())
        Serial.read(); // Esvazia o buffer.
    }
    delay(100);
  }

  String biases = f.readString();
  Serial.println(biases);

  while(f.available()){
    instant = f.readString();
    state_vector1 = f.readString();
    state_vector2 = f.readString();
    measurements_with_bias = f.readString();

    // state_vector1 e state_vector2 são printados numa única linha.
    Serial.println(instant);
    Serial.print(state_vector1); Serial.print(","); 
    Serial.println(state_vector2);
    Serial.println(measurements_with_bias);
  }

  f.close();
}

void loop() {
  // Nada aqui.
}
