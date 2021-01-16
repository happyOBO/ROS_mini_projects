/*
 * HC-SR04 ultrasonic FW
 */

 #define trig1 2 // define trig pin
 #define echo1 3 // define echo pin
 #define trig2 4 // define trig pin
 #define echo2 5 // define echo pin 
 #define trig3 6 // define trig pin
 #define echo3 7 // define echo pin
 #define trig4 8 // define trig pin
 #define echo4 9 // define echo pin
 
 void setup()
 {
    // start serial with 9600bps speed
    Serial.begin(9600);
    // define trig pin to output
    pinMode(trig1,OUTPUT);
    // define echopin to input
    pinMode(echo1,INPUT);
    // define trig pin to output
    pinMode(trig2,OUTPUT);
    // define echopin to input
    pinMode(echo2,INPUT);
    // define trig pin to output
    pinMode(trig3,OUTPUT);
    // define echopin to input
    pinMode(echo3,INPUT);
    // define trig pin to output
    pinMode(trig4,OUTPUT);
    // define echopin to input
    pinMode(echo4,INPUT);
   

 }

 void loop()
 {
    long duration1, distance1; // def var for distance
    long duration2, distance2; // def var for distance
    long duration3, distance3; // def var for distance
    long duration4, distance4; // def var for distance
    // print purse during 10us
    digitalWrite(trig1, LOW);
    delayMicroseconds(2); //2us delay
    digitalWrite(trig1, HIGH);
    delayMicroseconds(10); //10us delay
    digitalWrite(trig1,LOW);
    duration1 = pulseIn(echo1,HIGH);
    distance1 = duration1 * 170 / 1000;
    
    // print purse during 10us
    digitalWrite(trig2, LOW);
    delayMicroseconds(2); //2us delay
    digitalWrite(trig2, HIGH);
    delayMicroseconds(10); //10us delay
    digitalWrite(trig2,LOW);
    duration2 = pulseIn(echo2,HIGH);
    distance2 = duration2 * 170 / 1000;

    
    // print purse during 10us
    digitalWrite(trig3, LOW);
    delayMicroseconds(2); //2us delay
    digitalWrite(trig3, HIGH);
    delayMicroseconds(10); //10us delay
    digitalWrite(trig3,LOW);
    duration3 = pulseIn(echo3,HIGH);
    distance3 = duration3 * 170 / 1000;

    // print purse during 10us
    digitalWrite(trig4, LOW);
    delayMicroseconds(2); //2us delay
    digitalWrite(trig4, HIGH);
    delayMicroseconds(10); //10us delay
    digitalWrite(trig4,LOW);
    duration4 = pulseIn(echo4,HIGH);
    distance4 = duration4 * 170 / 1000;

    //pulseln() read pin sign and convert us


    //Serial.print("Distance(mm): \n");
    Serial.print(distance1);
    Serial.print("mm ");
    Serial.print(distance2);
    Serial.print("mm ");
    Serial.print(distance3);
    Serial.print("mm ");
    Serial.print(distance4);
    Serial.print("mm\n");
    delay(100);
 }
