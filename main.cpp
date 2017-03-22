#include "mbed.h"
#include "rtos.h"
#include "PID.h"
//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

#define PI 3.141592653589793

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
Serial pc(SERIAL_TX, SERIAL_RX);
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);


int8_t orState = 0;    //Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;
int i=0;
int j=0;
int buffer_time[3];
int buffer_speed[3];
int temp=0;
float rotations_user=600;

int rotations_completed=0;
float required_velocity=10;       //Input velocity from user       
float current_velocity=0;      //Measured velocity of motor
float Threshold=0;
Timer timer;
Thread thread;


float Kp=1;
float tauD=0.001;
float tauI=0.01;
float duty_cycle=1.0f;
float checkfreq=0.033;
PID speed_controller(Kp,tauI,tauD,checkfreq);
float wait_time=1;              //Time to wait between chages of state

void Velocity_Control(void);

//////////////////////////////////////////////////////////////////////////////////////////////////
//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H.write(duty_cycle);
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H.write(duty_cycle);
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H.write(duty_cycle);
    
    //Then turn on
    if (driveOut & 0x01) L1L.write(duty_cycle);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.write(duty_cycle);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.write(duty_cycle);
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

void stop_motor(){
    __disable_irq();
    motorOut(0);
    wait (1.0);
    __enable_irq();    
}


void Velocity_Measurement(){
    current_velocity= 1000/buffer_speed[i];
}

void Distance_Control(){
    if (rotations_completed>=rotations_user){
        stop_motor();
    }
}




void Update_State(){
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
}
    
void Counting(){
    temp= timer.read_ms();
    buffer_speed[j]=abs(buffer_time[j]-temp);
    buffer_time[j]=temp;
    Velocity_Measurement();
    j++;
    Distance_Control();
    if(j>2){
        j=0;
        rotations_completed++;
    }
} 


 

///////////////////////////////////

//Main
int main() {
    
    

    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    */
    
    for(i=0; i<3; i++){
        buffer_time[i]=0;
        buffer_speed[i]=0;
    }
    rotations_completed=0;
    timer.start();
    for(i=0;i<5;i++){ 
        Update_State(); 
    }
    speed_controller.setInputLimits(0.0,current_velocity);
    speed_controller.setOutputLimits(0.6f, 1.0f);
    
    I1.rise(&Counting);
    I2.rise(&Counting);
    I3.rise(&Counting);
    
    while(1){
        Update_State();
        }
}
