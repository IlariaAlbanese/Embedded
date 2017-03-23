#include "mbed.h"
#include "rtos.h"
#include "PID.h"
#include "mbed_memory_status.h"
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
DigitalOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

int8_t orState = 0;    //Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;
int i=0;
//int j=0;
int buffer_time;
int buffer_speed;
int temp=0;
float rotations_user=50;
int frequency= 440;

volatile int rotations_completed=0;
volatile float required_velocity=0.7f;       //Input velocity from user       
float current_velocity=0;      //Measured velocity of motor
float Threshold=0;
Timer timer;

float VKc=0.2;
float VtauD=0.01;
float VtauI=1;
float duty_cycle=1.0f;
float Vinterval= 0.01;
float Vout=0.6f;
//float wait_time=1;              //Time to wait between chages of state

//void Velocity_Control(void);

//////////////////////////////////////////////////////////////////////////////////////////////////
//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H.write(Vout);
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H.write(Vout);
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H.write(Vout);
    
    //Then turn on
    if (driveOut & 0x01) L1L.write(Vout);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.write(Vout);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.write(Vout);
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
    current_velocity= 1000/buffer_speed[j];
}

void Distance_Control(){
    led1=1;
    if ((rotations_user-rotations_completed)<5){
        velocity_required=0;
    }
    if (rotations_completed>=rotations_user){
        stop_motor();
    }
}

PID velocityControl(VKc, VtauI, VtauD, Vinterval);

void PIDinit(void){
    velocityControl.setInputLimits(0.0,current_velocity);
    velocityControl.setOutputLimits(0.58f, 1.0f);
    velocityControl.setTunings(VKc, VtauI, VtauD);
    velocityControl.setInterval(Vinterval);
    velocityControl.setSetPoint(required_velocity);
    velocityControl.setMode(1); 
}


void velocityPID(void){
    while(1){
        velocityControl.setProcessValue(current_velocity);
        Vout= velocityControl.compute();
        Thread::wait(Vinterval);
    }
}

void ISR(){
    Update_State();
    if (intState==0) {
    Counting();
    }
}
void Update_State(){
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
}
    
void Counting(){
    //led1=1;
    //Update_State();
    temp= timer.read_ms();
    buffer_speed=abs(buffer_time-temp);
    buffer_time=temp;
    Velocity_Measurement();
    Distance_Control();
    rotations_completed++;
    }
} 
void User_read(void);
Thread* userReadThread;
Thread* velocityControlThread;
int main() {
    PIDinit();

    //pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    //pc.printf("Rotor origin: %x\n\r",orState);
    
    buffer_time=0;
    buffer_speed=0;
    
    timer.start();
    
    //pc.printf("Setting up pwm");
        L1H.period(1/frequency);
        L2H.period(1/frequency);
        L2H.period(1/frequency);    
    
    
    for(int k=0;k<5;k++){ 
        Update_State();
    }

    
    I1.rise(&ISR);
    I2.rise(&ISR);
    I3.rise(&ISR);
    velocityControlThread=new Thread(osPriorityNormal, 256);
    velocityControlThread->start(&velocityPID);
    userReadThread=new Thread(osPriorityNormal);
    userReadThread->start(&User_read);
    while(1){
         pc.printf("R is %f\n\r",rotations_completed);
         pc.printf("V is %f\n\r",current_velocity);   
        }
}

void User_read(void){
        while(1){
        if(pc.readable()){
            char temp1=pc.getc();
            if (temp1=='R'){
                pc.scanf("%5f",&rotations_user);
            }
            else if(temp1=='V'){
                pc.scanf("%5f",&required_velocity);
            }
        }
        Thread::wait(1);
    }
}
