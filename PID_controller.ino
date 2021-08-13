//Proyecto 2 - Controlador PID

#include "Arduino.h"
#include <Schedulerrt.h>
#include <TimerOne.h>

// Entradas
#define pinCO A0 // pin entrada manual, analog
#define pinR A1 // pin referencia o setpoint, analog
#define pinY A2 // pin de salida realim, anlaog
#define pinModo A3 // pin de modo, digital

// Salidas
#define pinU 3 // pin de salida controlador, analog
#define pinLED_Modo 12 // pin de cambio modo, digit encendido=manual

// Tiempo de muestreo
float Ts = 0.1; //0.1 segundos
float Ts_ms = 100; //100 ms
float Ts_serial=0; //para obtener tiempo de asentamiento en puerto serial

// Variables
float R = 0; // setpoint
float CO = 0;
float Y = 0; 
float u = 0; 
float P=0;
float I=0;
float D=0;
float I_ant=0;
float D_ant=0;
float Y_ant=0;
float e=0; //señal de error
float e_ant=0;
// parametros controlador PI Huang&Jeng
float alfa=0.1;
float beta=1;
float kp=8.678;
float Ti=13.2024;
float Td=0;

Scheduler scheduler;

float salida_controlador; //señal de salida PWM

/************************************
* Callback functions *
************************************/

void salida() {
    
    //lectura entradas
    Y = analogRead(pinY);
    R = analogRead(pinR);
    CO = analogRead(pinCO);
    
    //modo automático
  if(digitalRead(pinModo)==LOW){

    //señalamiento de modo 
   digitalWrite(pinLED_Modo, LOW);
   
    //algoritmo PID
    e=R-Y;
    P=kp*(beta*R-Y);
    I=I_ant+(kp*Ts*e_ant)/Ti;
    D=(alfa*Td*D_ant)/(alfa*Td+Ts)-(kp*Td*(Y-Y_ant))/(alfa*Td+Ts);
    u=P+I+D;
    
    //antiwindup
    if (u>1023){
      u=1023;
    }
    else if (u<0){
      u=0;
    }   
    //valores anteriores
    I_ant=I;
    D_ant=D;
    Y_ant=Y;
    e_ant=e;
    
  } // cierra modo auto
  
  //modo manual
else if(digitalRead(pinModo)==HIGH){ 

  //señalamiento de modo
  digitalWrite(pinLED_Modo, HIGH);
  
  //salida directa
  u=CO;
  
} //cierra modo manual

//Visualización/graficación de señales
//para ver plotter quitar las barras (|) y el tiempo
    Serial.print(Ts_serial);
    Serial.print('\t');
    Serial.print('|');
    Serial.print('\t');
    Serial.print(Y*100/1023);
    Serial.print('\t'); 
    Serial.print(R*100/1023);
    Serial.print('\t'); 
    Serial.print(CO*100/1023);
    Serial.print('\t');
    Serial.print('|');
    Serial.print('\t');
    Serial.print(Y*5/1023);
    Serial.print('\t');
    Serial.print(R*5/1023);
    Serial.print('\t'); 
    Serial.print(CO*5/1023);
    Serial.print('\t');
    Serial.print('|');
    Serial.print('\t');
    Serial.print(u*100/1023);
    Serial.print('\t');
    Serial.println(u*5/1023);
    
// Escritura de entrada a la planta
// Se pasa el nivel de salida a escala de 0-255 bits
analogWrite(pinU, u*255/1023);
// Conteo del tiempo
Ts_serial=Ts_serial+0.1;
}//cierra salida()


void timerCallbackScheduler() {
scheduler.advanceScheduler();
}


/************************************
* Init
************************************/
void setup() {
  //definiciones
pinMode(pinU, OUTPUT);
pinMode(pinLED_Modo, OUTPUT);
Serial.begin(9600);

//Corre lazo auto/manual cada Ts:
 salida_controlador = scheduler.createSchedule(Ts_ms, -1, false, salida);

// Config. de timer
Timer1.initialize(1000); // Timer de 1ms
Timer1.attachInterrupt(timerCallbackScheduler);
sei();
}

/**************************************************************************
* Main Loop                                                               *
**************************************************************************/

void loop() {
scheduler.serviceScheduledEvents();
}
