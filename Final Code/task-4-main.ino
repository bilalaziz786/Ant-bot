/*
* Team Id : 2121
* Author List : Tamsil Sajid Amani, Mohd Bilal Aziz, Mohammed Omar Siddiqui, Zaid Hasan
* Filename: task-4-main.ino
* Theme: Ant Bot (AB)
* Functions: setup, servo_pick, servo_place, lineSensor, loop, setColor, turn_on_robot
* Global Variables: analogPin1, analogPin2, analogPin3, value1, value2, value3, stservo, micservo, Buzzer, redPin, greenPin, bluePin, flag
*/


#include <Servo.h>

int analogPin1 = 0;  // analogPin1: associated with right photo-receptor of White Line Sensor
int analogPin2 = 2;  // analogPin1: associated with middle photo-receptor of White Line Sensor
int analogPin3 = 4;  // analogPin1: associated with left photo-receptor of White Line Sensor
int value1 = 0;  //value1: stores the sensor value of right photo-receptor
int value2 = 0;  //value2: stores the sensor value of middle photo-receptor
int value3 = 0;  //value3: stores the sensor value of left photo-receptor
Servo stservo;  // stservo: variable of type Servo for Standard Servo motor
Servo micservo;  // micservo: variable of type Servo for Micro servo motor
int Buzzer = 13;  // Buzzer: assigned to pin number 13 of arduino for Buzzer sound

int redPin = 9;  // redPin: red transmitter of LED light is connected to pin 9 of arduino
int greenPin = 10;  // greenPin: green transmitter of LED light is connected to pin 10 of arduino
int bluePin = 11;  // bluePin: blue transmitter of LED light is connected to pin 11 of arduino


/*
*Function Name: setup
*Input: None
*Output: Base orientation of the servo motors and buzzer
*Logic: Opens serial port with a baud rate, attaches servo motors to respective pins of arduino
*Example Call: setup()
*/


void setup() {
    Serial.begin(9600);
    stservo.attach(3);
    micservo.attach(6);
    stservo.write(12);
    micservo.write(25);

    pinMode(Buzzer, OUTPUT);
}


/*
*Function Name: servo_pick
*Input: None
*Output: Picks up the supply/trash blocks
*Logic: Closes the micro servo and raises the standard servo in order
*Example Call: servo_pick()
*/


void servo_pick() {
    micservo.write(48);
    delay(1000);
    stservo.write(23);
    delay(1);
}


/*
*Function Name: servo_place
*Input: None
*Output: Puts down the supply/trash blocks
*Logic: Lowers the standard servo and opens the micro servo in order
*Example Call: servo_place()
*/


void servo_place() {
    stservo.write(12);
    delay(1000);
    micservo.write(25);
    delay(1000);
}


/*
*Function Name: lineSensor
*Input: None
*Output: prints the values obtained from the White Line Sensor
*Logic: uses 'analogRead()' function to get the sensor values
*Example Call: lineSensor()
*/


void lineSenor() {
    value1 = analogRead(analogPin1);     // read the input pin
    value2 = analogRead(analogPin2);     // read the input pin
    value3 = analogRead(analogPin3);     // read the input pin

    Serial.print(value1);
    Serial.print(" ");
    Serial.print(value2);
    Serial.print(" ");
    Serial.println(value3);

}


/*
*Function Name: loop
*Input: None
*Output: calls the 'lineSensor()' function repeatedly to get the sensor values And also calls the 'setColor' function to glow LED light
*Logic: checks if it needs to glow LED or not and then continue to get sensor values
*Example Call: loop()
*/


void loop() {
    if (Serial.available() > 0) {
        char serIn = Serial.read();
        if (serIn == 'A') {
            setColor(0, 0, 255); //red
        } else if (serIn == 'B') {
            setColor(0, 255, 0);  //green
        } else if (serIn == 'C') {
            setColor(255, 0, 0);  //blue
        } else if (serIn == 'D') {
            setColor(0, 255, 255);  //yellow
        } else if (serIn == 'P') {
            servo_pick();
        } else if (serIn == 'Q') {
            servo_place();
        } else if (serIn == 'Z') {
            turn_on_buzzer();
        }

        while (Serial.available() > 0)
            serIn = Serial.read();

    }

    lineSenor();
}


/*
*Function Name: setColor
*Input: red -> value for red segment of the LED,
 *      green -> value for green segment of the LED,
 *      blue -> value for blue segment of the LED,
*Output: turns on the respective LED color for 1 second
*Logic: calls 'analogWrite()' function to turn ON and turn OFF the LED
*Example Call: setColor(255,255,0)
*/

void setColor(int red, int green, int blue) {
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
    delay(1000);
    analogWrite(redPin, 255);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255);
}


/*
*Function Name: turn_on_buzzer
*Input: None
*Output: turns on the buzzer for 5 seconds
*Logic: calls 'digitalWrite()' function to turn ON and turn OFF the buzzer
*Example Call: turn_on_buzzer()
*/



void turn_on_buzzer() {

    digitalWrite(Buzzer, HIGH);
    delay(5000);
    digitalWrite(Buzzer, LOW);
    delay(5000);
}

