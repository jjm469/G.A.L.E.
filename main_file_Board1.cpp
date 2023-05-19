#include "mbed.h"
#include "SLCD.h"
#include "tsi_sensor.h"

//========================================================================
// G.A.L.E. Code "GROUND-BASED ACTUATING LEGGED EXPLORER"
// * This file is for Board 1 which controls Legs 1 and 3
//========================================================================

// Local Parameters
#define SERVO_PERIOD 20
#define LEG_PERIOD 2
#define NUMBER_OF_POINTS 50*LEG_PERIOD
int ind;                                      // Index variable for traversing array

// Module/Component instanciations 
SLCD slcd;                                    // LCD Screen 
TSIAnalogSlider slider(PTB16, PTB17, 100);    // Touch Sensor            
Timer servo_t;                                // 20 ms Timer

PwmOut servo1_a(PTB0);                        // PWM Ports Board 1 (board to servos)
PwmOut servo1_b(PTB1);
PwmOut servo3_a(PTB2);
PwmOut servo3_b(PTB3);

DigitalIn start(PTC1);                        // Receiving Digital Pin
DigitalIn reverse(PTC2);                      // Receiving Reverse Pin

PwmOut gLed(LED_GREEN);                       // LEDs                          
PwmOut rLed(LED_RED);

// Servo Pulse Widths
int servo1_2a_angle[NUMBER_OF_POINTS] = {1554,1552,1549,1547,1544,1542,1539,1536,1533,1530,1527,1524,1521,1518,1515,1511,1508,1504,1501,1497,1494,1490,1486,1482,1478,1474,1470,1466,1462,1457,1453,1449,1444,1440,1435,1430,1426,1421,1416,1411,1407,1402,1397,1392,1387,1381,1376,1371,1366,1360,1355,1350,1344,1339,1333,1328,1322,1316,1311,1305,1299,1293,1287,1281,1275,1269,1263,1257,1251,1245,1239,1232,1226,1220,1213,1207,1232,1259,1287,1316,1347,1379,1414,1450,1488,1526,1564,1600,1632,1660,1682,1698,1707,1708,1703,1691,1673,1650,1622,1590};
int servo1_2b_angle[NUMBER_OF_POINTS] = {1207,1213,1220,1226,1232,1239,1245,1251,1257,1263,1269,1275,1281,1287,1293,1299,1305,1311,1316,1322,1328,1333,1339,1344,1350,1355,1360,1366,1371,1376,1381,1387,1392,1397,1402,1407,1411,1416,1421,1426,1430,1435,1440,1444,1449,1453,1457,1462,1466,1470,1474,1478,1482,1486,1490,1494,1497,1501,1504,1508,1511,1515,1518,1521,1524,1527,1530,1533,1536,1539,1542,1544,1547,1549,1552,1554,1590,1622,1650,1673,1691,1703,1708,1707,1698,1682,1660,1632,1600,1564,1526,1488,1450,1414,1379,1347,1316,1287,1259,1232};
int servo3_4a_angle[NUMBER_OF_POINTS] = {1360,1355,1350,1344,1339,1333,1328,1322,1316,1311,1305,1299,1293,1287,1281,1275,1269,1263,1257,1251,1245,1239,1232,1226,1220,1213,1207,1232,1259,1287,1316,1347,1379,1414,1450,1488,1526,1564,1600,1632,1660,1682,1698,1707,1708,1703,1691,1673,1650,1622,1590,1554,1552,1549,1547,1544,1542,1539,1536,1533,1530,1527,1524,1521,1518,1515,1511,1508,1504,1501,1497,1494,1490,1486,1482,1478,1474,1470,1466,1462,1457,1453,1449,1444,1440,1435,1430,1426,1421,1416,1411,1407,1402,1397,1392,1387,1381,1376,1371,1366};
int servo3_4b_angle[NUMBER_OF_POINTS] = {1470,1474,1478,1482,1486,1490,1494,1497,1501,1504,1508,1511,1515,1518,1521,1524,1527,1530,1533,1536,1539,1542,1544,1547,1549,1552,1554,1590,1622,1650,1673,1691,1703,1708,1707,1698,1682,1660,1632,1600,1564,1526,1488,1450,1414,1379,1347,1316,1287,1259,1232,1207,1213,1220,1226,1232,1239,1245,1251,1257,1263,1269,1275,1281,1287,1293,1299,1305,1311,1316,1322,1328,1333,1339,1344,1350,1355,1360,1366,1371,1376,1381,1387,1392,1397,1402,1407,1411,1416,1421,1426,1430,1435,1440,1444,1449,1453,1457,1462,1466};

// Init LCD
void lcd_init(){
    slcd.printf("G.A.L.E");
}

// Sets all servos to the neutral position
void center_servos(){
    servo1_a.pulsewidth_us(1500);
    servo1_b.pulsewidth_us(1500);
    servo3_a.pulsewidth_us(1500);
    servo3_b.pulsewidth_us(1500);
}

// Sends the servos controlled by board 1 a pulsewidth from their respective arrays
void send_signals(void){
	servo1_a.pulsewidth_us(servo1_2a_angle[ind]);
	servo1_b.pulsewidth_us(servo1_2b_angle[ind]);
    servo3_a.pulsewidth_us(servo3_4a_angle[ind]);
    servo3_b.pulsewidth_us(servo3_4b_angle[ind]);
}

int main()  
{ 
    while(1){
        lcd_init();                                       // Init LCD with "G.A.L.E."
        ind = 0;                                          
        center_servos();                                  // Center Servos 

        while(start.read()==0);                           // Wait for signal from Board 2

        while(reverse.read()==0 && start.read()==1){      // While the Reverse button isn't pressed, and the signal from the master board is detected, 
            send_signals();                               // Update Servo PWM signals
            if (ind>=NUMBER_OF_POINTS-1){                 // If at the end of the PWM array, go back to the start 
                ind = 0;
            }
            else{
                ind++;                                    // Step through the array otherwise
            }
            servo_t.start();                              // Delay 
            while(servo_t.read_ms() <= SERVO_PERIOD);
            servo_t.reset();
            servo_t.stop();
        }

        center_servos();                                  // Center servos before going into reverse
        ind = NUMBER_OF_POINTS-1;                         // Set ind to the last element in all the arrays
        servo_t.start();                                  // Delay
        while(servo_t.read_ms() <= SERVO_PERIOD);
        servo_t.reset();
        servo_t.stop();

        while(start.read()==1){                           // While G.A.L.E. set to go in reverse
            send_signals();                               // Send PWM signals
            if (ind<=0){                                  // If at the start of the PWM array, go back to the end 
                ind = NUMBER_OF_POINTS-1;                 
            }
            else{
                ind--;                                    // We decrement because we traverse the array backwards
            }
            servo_t.start();                              // Delay
            while(servo_t.read_ms() <= SERVO_PERIOD);
            servo_t.reset();
            servo_t.stop();
        }
    }
}
