#include <checksum.h>
#include <mavlink.h>
#include <mavlink_helpers.h>
#include <mavlink_types.h>
#include <protocol.h>


#define Pecho1 12
#define Ptrig1 13

#define Pecho2 7
#define Ptrig2 8

#define Vout2 2
#define Vout1 3

long duracion, distancia;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(Pecho1, INPUT);
  pinMode(Ptrig1, OUTPUT);

  //pinMode(Pecho2, INPUT);
  //pinMode(Ptrig2, OUTPUT);

  pinMode(Vout1, OUTPUT);
  pinMode(Vout2, OUTPUT);
  
  //pinMode(13,1);
  
  pinMode(7,INPUT);
}

void loop() {
  digitalWrite(Vout1, HIGH);
  
  //digitalWrite(Vout2, HIGH);
  /*int duracion1 = pulseIn(7,HIGH);
  int distancia1 = (duracion1/147)*2.54;
  Serial.print("Sensor simple ");
  Serial.print(distancia1);
  Serial.println("cm");
  */
  // put your main code here, to run repeatedly:

  // Read from sensor 12 & 13
  digitalWrite(Ptrig1, LOW);
  delayMicroseconds(2);
  digitalWrite(Ptrig1,HIGH);
  delayMicroseconds(10);
  digitalWrite(Ptrig1,LOW);
  
  duracion = pulseIn(Pecho1, HIGH);
  distancia = (duracion/2)/29;

  
  Serial.print("Sensor 12 & 13 ");
  Serial.print(distancia);
  Serial.println("cm");
  digitalWrite(12,0);

  delay(100);
  /*
  // Read from sensor 7 & 8
  digitalWrite(Ptrig2, LOW);
  delayMicroseconds(2);
  digitalWrite(Ptrig2,HIGH);
  delayMicroseconds(10);
  digitalWrite(Ptrig2,LOW);
  
  duracion = pulseIn(Pecho2, HIGH);
  distancia = (duracion/2)/29;

  
  Serial.print("Sensor 7 & 8 ");
  Serial.print(distancia);
  Serial.println("cm");
  digitalWrite(7,0);
  */
  
  int duracion1 = pulseIn(7,HIGH);
  int distancia1 = (duracion1/147)*2.54;
  Serial.print("Sensor simple ");
  Serial.print(distancia1);
  Serial.println("cm");
  
  
  delay(1000);
}
