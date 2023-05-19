#include "mbed.h"
#include "SLCD.h"
#include "tsi_sensor.h"

//========================================================================
// G.A.L.E. Code "GROUND-BASED ACTUATING LEGGED EXPLORER"
// * This file is for Board 2 (master) which controls Legs 2 and 4
// and sends signals to Board 1
//========================================================================

// Local Parameters
#define SERVO_PERIOD 20
#define LEG_PERIOD 2
#define NUMBER_OF_POINTS 50*LEG_PERIOD

int ind;                                      // Index variable for traversing array

// Module/Component instanciations 
SLCD slcd;                                    // LCD Screen 
TSIAnalogSlider slider(PTB16, PTB17, 100);    // Touch sensor
Timer start;                                  // Start Timer (used with start button)
Timer servo_t;                                // 20 ms Timer
Timer reverse;                                // Reverse Timer

PwmOut servo2_a(PTB0);                        // PWM Ports Board 1 (board to servos)
PwmOut servo2_b(PTB1);
PwmOut servo4_a(PTB2);
PwmOut servo4_b(PTB3);

DigitalOut start_board1(PTC1);                // Outgoing Digital Pin (Start)
DigitalOut reverse_board1(PTC2);              // Outgoing Digital Pin (Reverse)

PwmOut gLed(LED_GREEN);                       // LEDs                     
PwmOut rLed(LED_RED);
 
InterruptIn start_button(PTC12);              // Buttons
InterruptIn reverse_button(PTC3);

// Start Button Handler
void start_button_handler() {
    start.start();
}

// Reverse Button Handler
void reverse_button_handler() {
    reverse.start();
}

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
    servo2_a.pulsewidth_us(1500);
    servo2_b.pulsewidth_us(1500);
    servo4_a.pulsewidth_us(1500);
    servo4_b.pulsewidth_us(1500);
}

// Sends the servos controlled by board 2 a pulsewidth from their respective arrays
void send_signals(void){
	servo2_a.pulsewidth_us(servo1_2a_angle[ind]);
	servo2_b.pulsewidth_us(servo1_2b_angle[ind]);
    servo4_a.pulsewidth_us(servo3_4a_angle[ind]);
    servo4_b.pulsewidth_us(servo3_4b_angle[ind]);
}

int main()  
{ 
    lcd_init();                                       // Init LCD with "G.A.L.E."
    start_board1.write(0);                            // Ensure digital out pins are sending 0
    reverse_board1.write(0);

    start_button.fall(&start_button_handler);         // Initialize Interrupt Handlers
    reverse_button.fall(&reverse_button_handler);

    ind = 0;                                         
    center_servos();                                  // Center Servos 

    while(start.read_ms()==0);                        // Wait for start button

    start_board1.write(1);                            // Start Board 1 by writing high value to digital out

    while(reverse.read_ms()==0){                      // While the reverse button has not been pressed
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

    reverse_board1.write(1);                          // Once the reverse button has been pressed, signal board 1 to go in reverse
    center_servos();                                  // Center Servos 
    ind = NUMBER_OF_POINTS-1;                         // Set ind to the last element in all the arrays
    servo_t.start();                                  // Delay
    while(servo_t.read_ms() <= SERVO_PERIOD);
    servo_t.reset();
    servo_t.stop();

    while(1){
        send_signals();                               // Update Servo PWM signals
        if (ind<=0){                                  // If at the start of the PWM array, go back to the end 
            ind = NUMBER_OF_POINTS-1;
        }
        else{  
            ind--;                                    // We decrement because we traverse the array backwards otherwise
        }   
        servo_t.start();                              // Delay
        while(servo_t.read_ms() <= SERVO_PERIOD);
        servo_t.reset();
        servo_t.stop();
    }
}

