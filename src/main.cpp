//#include <ps4.h>
#include <PS4Controller.h>
//#include <ps4_int.h>

//#include <ESP32Servo.h>
#include <SPI.h>
//#include <Wire.h>

//#include <VarSpeedServo.h>
#include <Adafruit_PWMServoDriver.h>

//Bibliotecas para remover controles pareados
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

PS4Controller controller;

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define MIN_PULSE_WIDTH 500  //650      //Recomendado colocar entre 500 e 600
#define MAX_PULSE_WIDTH 2500 //2350     //Recomendado colocar entre 2400 e 2500
#define DEFAULT_PULSE_WIDTH 1500 //Pulso para parar o servo em um determinado ângulo
#define FREQUENCY 50             //Frequência de operação do Servo

#define MAC_CONTROLLER "b0:8b:f6:5f:4e:56" // MAC do PS4: "b0:05:94:65:55:5e" MAC Gerado: "b0:8b:f6:5f:4e:56"

#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define CLAW 3

int clawPosition = 0;
int elbowPosition = 0 ;
int shoulderPosition = 0;
int basePosition = 0;

// Recebe um valor de ângulo em graus e converte em um valor de pulso para a placa
int angleToPulse(int angulo){
   int pulseWide = map(angulo, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); //Mapeia o ângulo recebido para valores que a PCA9685 consegue ler (0 a 4096), mas, no caso desses servos, conforme os valores definidos
   int analogValue = int(float(pulseWide) / 1000000 * FREQUENCY * 4096);
   Serial.print("Ângulo: ");Serial.println(angulo);
   Serial.print("Largura do Pulso: ");Serial.println(pulseWide);
   Serial.print("Valor PWM: ");Serial.println(analogValue);
   
   return analogValue;
}

void removePairControllers() {
  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for(int i = 0; i < count; i++) 
  {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void setup(){

  controller.begin(MAC_CONTROLLER);

  //Remove controles pareados
  removePairControllers();

  Serial.begin(9600);

  //Laço de espera da conexão do controle
  while (!controller.isConnected()) {
    Serial.println("Esperando Conexao");
    delay(250);
  }
  
  controller.setLed(100, 0, 0); //Define a cor do LED
  controller.sendToController();// Envia informações para o controle (a cor do led, nesse caso)
  
  Serial.println("Conexão Estabelecida");

  board1.begin(); //Inicialização da PCA9685
  board1.setPWMFreq(FREQUENCY); //Define a frequência em que a placa vai trabalhar
  board1.setPWM(CLAW, 0, DEFAULT_PULSE_WIDTH); //Coloca a garra na posição inicial
  board1.setPWM(ELBOW, 0, DEFAULT_PULSE_WIDTH); //Coloca o cotovelo na posição inicial
  board1.setPWM(SHOULDER, 0, DEFAULT_PULSE_WIDTH); //Coloca o ombro na posição inicial
  board1.setPWM(BASE, 0, DEFAULT_PULSE_WIDTH); //Coloca a base na posição inicial
}

void loop(){
  if (controller.isConnected()) {
   //Serial.print("Pulso: ");Serial.println(controller.R2Value());
    int clawValueOpen = controller.R2Value();
    int clawValueClose = controller.L2Value();
    int elbowValue = controller.RStickY();
    int shoulderValue = controller.LStickY();
    int baseValue = controller.LStickX();

    if (clawValueOpen > 10 && clawPosition <= 255){
      while (clawValueOpen > 10)
      {
        clawPosition = clawPosition +3;
        clawPosition > 255 ? clawPosition = 255 : clawPosition;
        int clawPulseOpen = angleToPulse(map(clawPosition, 0, 255, 0, 180)); //Mapeia os pulsos de 0 a 255 do analógico R2 para ângulos de 0º a 180º
        board1.setPWM(CLAW, 0, clawPulseOpen);
        clawValueOpen = controller.R2Value();
      }      
    } else if (clawValueClose > 10 && clawPosition <= 255){
      clawPosition = clawPosition - 50;
      clawPosition < 0 ? clawPosition = 0 : clawPosition;
      int clawPulseClose = angleToPulse(map(clawPosition, 0, 255, 0, 180)); //Mapeia os pulsos de 0 a 255 do analógico L2 para ângulos de 0º a 180º
      board1.setPWM(CLAW, 0, clawPulseClose);
    }
    int elbowPulse = angleToPulse(map(elbowValue, -128, 127, 10, 170)); //Mapeia os pulsos de -128 a 127 do eixo y do analógico direito para ângulos de 0º a 180º
    int shoulderPulse = angleToPulse(map(shoulderValue, -128, 127, 10, 170)); //Mapeia os pulsos de -128 a 127 do eixo y do analógico esquerdo para ângulos de 0º a 180º
    int basePulse = angleToPulse(map(baseValue, -128, 127, 10, 170)); //Mapeia os pulsos de -128 a 127 do eixo x do analógico esquerdo para ângulos de 0º a 180º
    //if (pulse > 1800 && pulse < 2500) {
      //board1.setPWM(1, 0, 2048);
    //} else {
    board1.setPWM(ELBOW, 0, elbowPulse);
    board1.setPWM(SHOULDER, 0, shoulderPulse);
    board1.setPWM(BASE, 0, basePulse);
    }
}