#include <Servo.h>
int StringCount = 0;

Servo flm;
Servo frm;
Servo rlm;
Servo rrm;
Servo frlm;
Servo frrm;

void setup() {
  Serial.begin(9600);
  flm.attach(3);
  frm.attach(5);
  rlm.attach(6);
  rrm.attach(9);
  frlm.attach(10);
  frrm.attach(11); 

  frm.write(0);
  rrm.write(0);
  flm.write(0);
  rlm.write(0);
  frrm.write(0); 
  frlm.write(0);
}

void loop() {
  while(Serial.available()>0){
    char data = Serial.read();
    switch(data){
      case 'm':
        float val1, val2, val3, val4, val5, val6;
        int spaceIndex = 0;
        int valueCount = 0;

        String inputString = Serial.readStringUntil('\n'); //read string until new line
        inputString.trim(); //trim any leading or trailing spaces

        //split string into individual floats
        while (spaceIndex < inputString.length() && valueCount < 6) {
          int nextSpace = inputString.indexOf(' ', spaceIndex);
          if (nextSpace == -1) nextSpace = inputString.length();
          
          String value = inputString.substring(spaceIndex, nextSpace);
          
          switch(valueCount) {
            case 0: val1 = value.toFloat(); break;
            case 1: val2 = value.toFloat(); break;
            case 2: val3 = value.toFloat(); break;
            case 3: val4 = value.toFloat(); break;
            case 4: val5 = value.toFloat(); break;
            case 5: val6 = value.toFloat(); break;
          }

          valueCount++;
          spaceIndex = nextSpace + 1;
        }
        //insert motor control from values here
        frm.write(val1);
        rrm.write(val2);
        flm.write(val3);
        rlm.write(val4);
        frrm.write(val5); 
        frlm.write(val6);

        // Serial.println("OK"); //print out values
        Serial.println("motors: " + String(val1) + " " + String(val2) + " " + String(val3) + " " + String(val4) + " " + String(val5) + " " + String(val6));
        break;
      default:
        Serial.print("Option unavailable\r\n");
        break;
    }
    
  }
}
