#include <WiFi.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
MAX30105 medicSensor;
#define Port 3000
const char* ssid = " ";
const char* password = " ";

WiFiServer server(Port);

#define USEFIFO

double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//interwał po któreym następuje przeliczenie spo2
 
double ESpO2 = 95.0;//wartośc początkowa SpO2
double FSpO2 = 0.7; //estymacja filtrem dla  SpO2
double frate = 0.95; //komponent DP filtra dla elimiancji zaklocen
#define TIMETOBOOT 3000 // po 3 sek zostanie podana wartosc spo2
#define SCALE 88.0 //skalowanie hr dla kreslarki
#define SAMPLING 1 //poziom dokladnsoci wizualizacji hr na kreslarce, ustawienie od 1 do 10
#define FINGER_ON 30000 // prog po ktorym alogrytm uznaje ze polozyes palca 
#define MINIMUM_SPO2 80.0

const byte RATE_SIZE = 8; //wspolczynnik usrednienia: wielkosc tablicy ktora bedzie filtrem usredniajacym
float rates[RATE_SIZE]; //tablica HR
int rateSpot = 0;
long lastBeat = 0; //kiedy wystapilo ostatnie bicie serca
float BPM; // wynik
float BPMAvg;// usredniony wynik hr

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Project, Please Wait...");
 
  // Initialize sensor
  if (!medicSensor.begin(Wire, I2C_SPEED_FAST)) // I2C  400kHz 
  {
    Serial.println("MAX30102 was not found. Make sure that u connected sensor  ");
    while (1);
  }
 
  //Setup to sense a nice looking saw tooth on the plotter sekcja znaleziona w datasheet
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  medicSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //konfiguracja 


}
void loop() {
  double SpO2 = 0; //SpO2 przed filtracja
   uint32_t ir, red , green;
  double fred, fir;
  
#ifdef USEFIFO
  medicSensor.check(); //
 
  while (medicSensor.available()) {//nowe dane ?
#ifdef MAX30105
   red = medicSensor.getFIFORed(); //Sparkfun's MAX30105
    ir = medicSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
    red = medicSensor.getFIFOIR(); //czerwone
    ir = medicSensor.getFIFORed(); //podczerwone
    #endif
    
    i++;
    
    fred = (double)red; // rzutowanie z uint32 na double
    fir = (double)ir; // rzutowanie z uint 32 na double
    avered = avered * frate + (double)red * (1.0 - frate);//czerwone usrednione, filtr dp
    aveir = aveir * frate + (double)ir * (1.0 - frate); //podczerwone usrednione, filtr dp
    sumredrms += (fred - avered) * (fred - avered); //suma kwadratow
    sumirrms += (fir - aveir) * (fir - aveir);//suma kwadratow


// SEKJCA DO UZYCIA KRESLARKI
if ((i % SAMPLING) == 0) {
      if ( millis() > TIMETOBOOT) {
        //
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //skalowanie dla kreslarki

        // 
        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < 80.0 ) red_forGraph = 80.0;
      // pomoc do fali pulsu
       
        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //wskazanie oderwania palca, niepotrzebne w celach naukowych ale mozna na tej podstawie wykryc czy saturacja spada czy palec sie zsunął
        Serial.print(ir_forGraph); // na kreslarce pokaze sie fala pulsu 
        Serial.print(","); Serial.print(red_forGraph); // to samo 
        Serial.print(",");
        Serial.print(ESpO2); // filtr DP dla SpO2
        //
        Serial.print(","); Serial.print(85.0); 
        Serial.print(","); Serial.print(90.0); 
        Serial.print(","); Serial.print(95.0); 
        Serial.print(","); Serial.println(100.0); 
        // linie pomocnicze na kresalrce
      }
    }
     if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; // algorytm z strony w linku 
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//Filtracja DP
        Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; 
      sumirrms = 0.0;
      i = 0;
      break;
    }
    medicSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2);
}
#else
 
  while (1) {// kolejne wywolanie przyjecia danych ten sam algorymt co wyzej
#ifdef MAX30105
   red = medicSensor.getRed();  //Sparkfun's MAX30105
    ir = medicSensor.getIR();  //Sparkfun's MAX30105
#else
    red = medicSensor.getIR(); 
    ir = medicSensor.getRed(); 
#endif
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);
    aveir = aveir * frate + (double)ir * (1.0 - frate); 
    sumredrms += (fred - avered) * (fred - avered); 
    sumirrms += (fir - aveir) * (fir - aveir);
    if ((i % SAMPLING) == 0) {
      //#if 0
      if ( millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
       
        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < 80.0 ) red_forGraph = 80.0;
       
        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; 
        Serial.print((2.0 * fir - aveir) / aveir * SCALE); 
        Serial.print(","); Serial.print((2.0 * fred - avered) / avered * SCALE); 
        Serial.print(","); Serial.print(ESpO2); 
        Serial.print(","); Serial.print(85.0); 
        Serial.print(","); Serial.print(90.0); 
        Serial.print(","); Serial.print(95.0); 
        Serial.print(","); Serial.println(100.0); 
        //#endif
      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; 
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
        Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0;
      sumirrms = 0.0;
      i = 0;
      break;
    }
    medicSensor.nextSample(); 
   // Serial.println(SpO2);
  }
#endif
//Serial.print(BPM); 
//Serial.print("; "); 
Serial.print(BPMAvg);
long IrVal = medicSensor.getIR();
if(checkForBeat(IrVal) == true) {
  long dlt = millis() - lastBeat;
  lastBeat= millis();

  BPM = 60/(dlt/1000.0);

  if(BPM<270 && BPM>20)
  {
    rates[rateSpot++] = BPM;
    rateSpot%=RATE_SIZE;
    BPMAvg = 0;
    for(byte x =0; x<RATE_SIZE; x++)
      BPMAvg+= rates[x];
    BPMAvg/=RATE_SIZE;
  }
}

}
