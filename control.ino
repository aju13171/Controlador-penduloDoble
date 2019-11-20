#include <SPI.h>
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

/*
 * --------------- SYSTEM VARIABLES ---------------
 */
const float T = 800000;
const float TS_MICRO = 10000.0;

boolean debug1 = true; //print x0,x1,x2,x3
boolean debug2 = true; //print u1, u2
int count1 = 0, count2 = 0;
int t1 = 200;

/*
 * --------------- DAC block ---------------
 */
int dac1 = 4, dac2 = 3;
int cont = 0; //debug
const byte Gain = 1;

/*
 * --------------- LQR block ---------------
 */
float x[4];
int motor1 = A0, motor2 = A3; //PD-3 ; PE-2
//float offset1 = 1.71, offset2 = 1.45;
float offset1 = 1.72, offset2 = 1.52;
//float offset1 = 0.04, offset2 = 0.06;
float x_prev[2] = {0,0};

//float k1[4] = {11.8853, 0.1092, 4.5356, -0.2902};
//float k2[4] = {38.4495, -0.2838, 20.6129, -0.1693};

//float k1[4] = {7.6040, 0.0447, 2.9891, 0.1664};
//float k2[4] = {38.4298, -0.2907, 20.6025, -0.1630};

float k1[4] = {-12.8316, 1.0071, -14.9208, -0.9182};
float k2[4] = {39.9068, -0.2109, 20.7537, -0.2632};

float c1=0, c2=0; //control
float cota1 = 20.0, cota2 = 20.0; //cotas
int uk1 = 0, uk2 = 0;

/*
 * CONFIGURE TIMER
 * Función que configura el timer (1A en este ejemplo)
 */
void configureTimer1A(){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
  ROM_IntMasterEnable(); // Enable Interrupts
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic

  //time frequency
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, T); 

  TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1AHandler);
  ROM_IntEnable(INT_TIMER1A);  // Enable Timer 1A Interrupt
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer 1A Interrupt when Timeout
  ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Start Timer 1A
}

/*
 * --------------- SETUP ---------------
 */
void setup(){
  //SERIAL BEGIN
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  
  //SET PINS MODE
  pinMode(dac1, OUTPUT);
  pinMode(dac2, OUTPUT);
  digitalWrite(dac1, HIGH);
  digitalWrite(dac2, HIGH);

  //SPI BLOCK
  SPI.begin();

  //TIMER CONFIGURATION
  configureTimer1A();

  //SET PINS
  pinMode(motor1, INPUT_PULLUP);
  pinMode(motor2, INPUT_PULLUP);
}

void loop(){
}

void Timer1AHandler(void){
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A); //keep

  count1++;
  count2++;
  //--------------- CONTROLADOR
  x[0] = analogRead(motor1)*3.3/4095 - offset1;
  x[1] = (x[0] - x_prev[0])/(TS_MICRO/1000000.0);
  x[2] = analogRead(motor2)*3.3/4095 - offset2;
  x[3] = (x[2] - x_prev[1])/(TS_MICRO/1000000.0);

  if(debug1 and count1>=t1){
    count1 = 0;
    Serial.print("x0 = "); Serial.print(x[0]);
    Serial.print("\tx1 = "); Serial.print(x[1]);
    Serial.print("\tx2 = "); Serial.print(x[2]);
    Serial.print("\tx3 = "); Serial.println(x[3]);
  }
  
  c1 = -(k1[0]*x[0] + k1[1]*x[1] + k1[2]*x[2] + k1[3]*x[3]);
  c2 = -(k2[0]*x[0] + k2[1]*x[1] + k2[2]*x[2] + k2[3]*x[3]);

  x_prev[0] = x[0];
  x_prev[1] = x[2];

  if(c1 > cota1){c1 = cota1;}
  if(c1 < -cota1){c1 = -cota1;}
  if(c2 > cota2){c2 = cota2;}
  if(c2 < -cota2){c2 = -cota2;}

  uk1 = (int)((c1-(-cota1))*4095.0/(cota1-(-cota1)));
  uk2 = (int)((c2-(-cota2))*4095.0/(cota2-(-cota2)));
  
  if(debug2 and count2>=t1){
    count2 = 0;
    Serial.print("uk1 = "); Serial.print(uk1);
    Serial.print("\tuk2 = "); Serial.println(uk2);

    Serial.print("c1 = "); Serial.print(c1);
    Serial.print("\tc2 = "); Serial.println(c2);
    Serial.println("------\n");
  }
  DAC_Write(uk1, dac1);
  DAC_Write(uk2, dac2);
}

// Función para mandar un valor al DAC MCP4921
// Entradas: valor - entero con valores que deben estar entre 0 y 4095
//  slaveSelectPin - pin usado para el SS. El pin debe estar conectado
//                   al pin CS del DAC.
void DAC_Write(int valor, int slaveSelectPin) {
  // Baja el pin SS para seleccionar el chip
  digitalWrite(slaveSelectPin, LOW);  // No sería necesario, si sólo quiero
      // mandar un byte. La función SPI.transfer baja el pin 2 (por defecto)
      // y luego lo sube al terminar de mandar el byte. El problema es que
      // el chip MCP4921 tiene que recibir los dos bytes seguidos, sin que
      // el pin CS suba. De lo contrario, el primer byte se ignora.

  byte primero = (byte)(0b01010000 | (Gain << 5) | ((valor >> 8) & 0b00001111));
  byte segundo = (byte)(0x00FF & valor);
  SPI.transfer(primero); // 1er byte: configuración y 4 bits más significativos
  SPI.transfer(segundo); // 2do byte: 8 bits menos significativos

  // Sube el pin SS para liberar al chip
  digitalWrite(slaveSelectPin, HIGH);
}
