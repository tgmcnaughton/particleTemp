// particle photon code to poll up to 16 ds18b20 temperature probes,
// makes JSON strings of both temperature data eg:
//   {"t0":77.34,"t1":76.89,"t2":76.66,"t3":76.78,"t4":77.56,"t5":77.45}
//and probe addresses eg:
//   {"a0":"28442242060000fa","a1":"289c6641060000e1","a2":"28621e410 ...
// and "publishes" these strings to particle cloud
// also exposes the variables as "result" and "addresses"

#include "Particle-OneWire.h"
OneWire  ds(D0);  // a 1 - 4.7K resistor to 3.3v is necessary

#define DEBUG 1

unsigned long PUBLISH_INTERVAL = 1 * 60 * 1000UL;  //60 sec
unsigned long lastPublished = PUBLISH_INTERVAL;
unsigned long AWAITING_PROBE = 5 * 1000UL;  //5 sec
unsigned long lastProbeTime = AWAITING_PROBE;

bool getNextProbe = TRUE;
char bufferString[600];
char resultString[600];
char addresses[16][20];
char addressString[600];
char statusMsg[64];
//char errMsg[64];
char strBuf[32];
char version[4] = "2.0";
int probeNum = 0;
float t[16];

byte i;
byte type_s;
byte data[12];
byte addr[8];
float celsius, fahrenheit;
unsigned long now = millis();

int tinkerDigitalRead(String pin);
int tinkerDigitalWrite(String command);
int tinkerAnalogRead(String pin);
int tinkerAnalogWrite(String command);

void setup(void) {
    Serial.begin(57600);

    ds.reset();
    ds.reset_search();
    delay(250);

    strcpy(statusMsg,"Starting Particle data Logger");
    Spark.publish("statusMsg",statusMsg);
    //Serial.println("Started Ver:");  Serial.println(version);
    Spark.variable("result", resultString, STRING);  // makes variable result accessible via REST API
    // var response = UrlFetchApp.fetch("https://api.spark.io/v1/devices/DEVICEID/result?access_token=TOKEN");
    Spark.variable("addresses", addressString, STRING);
    //Spark.variable("version", version, STRING);
    //Spark.variable("errMsg", errMsg, STRING);

    Spark.function("digitalread", tinkerDigitalRead);
    Spark.function("digitalwrite", tinkerDigitalWrite);
    Spark.function("analogread", tinkerAnalogRead);
    Spark.function("analogwrite", tinkerAnalogWrite);

    pinMode(D7, OUTPUT);

    lastProbeTime = millis();
    lastPublished = millis();
}
void loop(void) {

    if (getNextProbe) {
        getNextProbe = FALSE;  // don't enter this loop unless getNextProbe flag get reset
        if ( ds.search(addr)) {  // returns address of next device on bus or 0 if none remaining
            if(DEBUG) { Serial.print(" ROM: "); for( i = 0; i < 8; i++) { Serial.write(' '); Serial.print(addr[i], HEX); }  }
            if (OneWire::crc8(addr, 7) != addr[7]) {
                Serial.println("CRC is not valid!");
                return;
            }
            switch (addr[0]) { // the first ROM byte indicates which chip
                case 0x28:
                Serial.print("  Chip = DS18B20 -> ");
                type_s = 0;
                break;
                default:
                Serial.println("Device is not a DS18B20 device. Skipping...");
                return;
            }
            Serial.printf("Probe: %d ",probeNum);
            strcpy(addresses[probeNum],"\"");  //copy this probe's address
            for( i = 0; i < 8; i++) {
                sprintf(strBuf,"%02x",addr[i]);
                strcat(addresses[probeNum],strBuf);
            }
            strcat(addresses[probeNum],"\"");
            Serial.print(addresses[probeNum]);
        }
        else  {   // no more devices on bus so collate results and publish
            ds.reset_search();
            delay(250);
            strcpy(bufferString,"{");
            for (i=0;i<probeNum-1;i++) {
                sprintf(strBuf,"\"t%d\":%4.2f,",i,t[i]);  // produces JSON: {"t0":79.23, "t2":78.45, .... }
                strcat(bufferString,strBuf);
            }
            sprintf(strBuf,"\"t%d\":%4.2f}",i,t[i]);  // last entry in JSON list ends with brace not comma.
            strcat(bufferString,strBuf);
            strcpy(resultString,bufferString);
            Serial.print("resultString: ");     Serial.println(resultString);

            strcpy(bufferString,"{");
            for (i=0;i<probeNum-1;i++) {
                sprintf(strBuf,"\"a%d\":%s,",i,addresses[i]);  // produces JSON: {"a0":12345678, "a1":45F3EA21, .... }
                strcat(bufferString,strBuf);
            }
            sprintf(strBuf,"\"a%d\":%s}",i,addresses[i]);  // last one ends with brace not comma.
            strcat(bufferString,strBuf);
            strcpy(addressString,bufferString);
            Serial.print("addressString:");     Serial.println(addressString);

            if (now-lastPublished > PUBLISH_INTERVAL ) {  //Every PUBLISH_INTERVAL
                lastPublished = now;
                Spark.publish("addresses",addressString);
                Spark.publish("result",resultString);
                strcpy(statusMsg,"Ver:");
                sprintf(strBuf," %s",version);
                strcat(statusMsg,strBuf);
                strcat(statusMsg," Probes #");
                sprintf(strBuf," %d",probeNum);
                strcat(statusMsg,strBuf);
                Spark.publish("statusMsg",statusMsg);
            }
            probeNum = 0;
        }
        ds.reset();
        ds.select(addr);
        ds.write(0x44);   // initiates a temperature measurement cycle
        lastProbeTime = now;
        //delay(900);
        digitalWrite(D7, HIGH);

    }

    now = millis();  // now is in milliseconds
    if (now-lastProbeTime > AWAITING_PROBE ) {  //won't enter this loop more than once each second.
                                                // this prevents blocking of tinker commands.
        getNextProbe = TRUE;
        digitalWrite(D7, LOW);

        ds.reset();
        ds.select(addr);
        ds.write(0xBE, 0);         // Read Scratchpad 0

        for ( i = 0; i < 9; i++) {           // we need 9 bytes
            data[i] = ds.read();
            if(DEBUG) { Serial.print(data[i], HEX); }
        }
        // Convert the data to actual temperature
        int16_t raw = (data[1] << 8) | data[0];
        celsius = (float)raw / 16.0;
        fahrenheit = celsius * 1.8 + 32.0;

        t[probeNum]=fahrenheit;
        probeNum=probeNum+1;  // increment probeNum only after full collection of data

        Serial.print("  T = ");
        Serial.print(celsius);  Serial.print(" C  / ");
        Serial.print(fahrenheit);  Serial.println(" F");
    }
}

///////////////////////////////////////////////////////////////////////////////
// Tinker specific functions below.
///////////////////////////////////////////////////////////////////////////////

int tinkerDigitalRead(String pin) {
    int pinNumber = pin.charAt(1) - '0';
    if (pinNumber< 0 || pinNumber >7) return -1;
    if(pin.startsWith("D")) {
        pinMode(pinNumber, INPUT_PULLDOWN);
        return digitalRead(pinNumber);
    }
    else if (pin.startsWith("A")){
        pinMode(pinNumber+10, INPUT_PULLDOWN);
        return digitalRead(pinNumber+10);
    }
    return -2;
}

int tinkerDigitalWrite(String command){
	bool value = 0;
	int pinNumber = command.charAt(1) - '0';
	if (pinNumber< 0 || pinNumber >7) return -1;
	if(command.substring(3,7) == "HIGH") value = 1;
	else if(command.substring(3,6) == "LOW") value = 0;
	else return -2;
	if(command.startsWith("D")){
		pinMode(pinNumber, OUTPUT);
		digitalWrite(pinNumber, value);
		return 1;
    }
	else if(command.startsWith("A")){
		pinMode(pinNumber+10, OUTPUT);
		digitalWrite(pinNumber+10, value);
		return 1;
    }
	else return -3;
}

int tinkerAnalogRead(String pin){
	int pinNumber = pin.charAt(1) - '0';
	if (pinNumber< 0 || pinNumber >7) return -1;
	if(pin.startsWith("D")){
		pinMode(pinNumber, INPUT);
		return analogRead(pinNumber);
    }
	else if (pin.startsWith("A")){
		pinMode(pinNumber+10, INPUT);
		return analogRead(pinNumber+10);
    }
	return -2;
}

int tinkerAnalogWrite(String command){
	int pinNumber = command.charAt(1) - '0';
	if (pinNumber< 0 || pinNumber >7) return -1;
	String value = command.substring(3);
	if(command.startsWith("D")){
		pinMode(pinNumber, OUTPUT);
		analogWrite(pinNumber, value.toInt());
		return 1;
    }
	else if(command.startsWith("A")){
		pinMode(pinNumber+10, OUTPUT);
		analogWrite(pinNumber+10, value.toInt());
		return 1;
    }
	else return -2;
}
