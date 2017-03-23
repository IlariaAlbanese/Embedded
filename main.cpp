#include "mbed.h"
#include "rtos.h"
#include "PID.h"
#include <cstdlib>
#include <string>
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

InterruptIn CHA(CHA);
InterruptIn CHB(CHB);

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
int increments=0;
float precision_rotations=0;
float rotations_user=50;
float frequency= 1.0;
int quadrature_state=0;
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

float RKc=0.2;
float RtauD=0.01;
float RtauI=1;
float R_duty_cycle=1.0f;
float VintervalR= 0.01;
float VoutR=0.6f;
int index;
int index_R=0;
int index_V=0;
bool found_R;
bool found_V;
char rot_string[5];
char vel_string[5];
//float wait_time=1;              //Time to wait between chages of state
char input_buffer[16];
int rot_index=0;
int vel_index=0;
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
    current_velocity= 1000/buffer_speed;
}

void PrecisionDistance_Control(){
    led1=1;
    if ((rotations_user-precision_rotations)<5){
        velocity_required=0;
    }
    if (precision_rotations>=rotations_user){
        stop_motor();
    }
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
    
    distanceControl.setInputLimits(0.0,rotations_completed);
    distanceControl.setOutputLimits(0.58f, 1.0f);
    distanceControl.setTunings(RKc, RtauI, RtauD);
    distanceControl.setInterval(VintervalR);
    distanceControl.setSetPoint(rotations_user);
    distanceControl.setMode(1);

}


void velocityPID(void){
    while(1){
        velocityControl.setProcessValue(current_velocity);
        Vout= velocityControl.compute();
        Thread::wait(Vinterval);
    }
}

void distancePID(void){
    while(1){
        distanceControl.setProcessValue(rotations_completed);
        VoutR= distanceControl.compute();
        Thread::wait(VintervalR);
    }
}

void smallest(){
    while(1){
        if(VoutR<=Vout){
            Vout=VoutR;
        }
    }
}

void ISR(){
    Update_State();
    if (intState==0) {
    Counting();
    }
}

void precision_ISR(void)
{
    quadrature_state = (CHA*2 + CHB);
    if(quadrature_state==0){
    Counting_precision();
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

void Counting_precision(){
    float position_degrees;
    
    increments++;
    position_degrees= increments*(360/117);
    if (position_degrees>360){
    position_degrees-=360;
    precision_rotations++;
    }
    precision_rotations+=position_degrees/360;
    PrecisionDistance_Control();
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
    User_read();
    pc.printf("R is %f\n\r", rotations_user);
    pc.printf("Velocity is %f\n\r", required_velocity);  
        }
}
void User_read(){
            if(pc.readable()){
            index=0;
            do{
                input_buffer[index]=pc.getc();
                pc.putc(input_buffer[index]);
                index++;
            }while(pc.getc() != 10 && pc.getc() != 13);
          pc.printf("Reading complete");
        found_R=false;
        found_V=false;
        //find the positions of R&V
        for(int i=0;i<=index;i++){
            if(input_buffer[i]=='R'){
                index_R=i;
                found_R=true;
                }
            if(input_buffer[i]=='V'){
                index_V=i;
                found_V=true;
                }
        }
        if(found_R==true&&found_V==true){
            rot_index=0;
            vel_index=0;
            for (int r=index_R+1;r<index_V-1;r++){
                rot_string[rot_index]=input_buffer[r];
                rot_index++;
                }
            for (int v=index_V+1;v<16;v++){
                vel_string[vel_index]=input_buffer[v];
                vel_index++;
                }
        }
        else if(found_R==true&&found_V==false){
            rot_index=0;
            for (int r=index_R+1;r<16;r++){
                rot_string[rot_index]=input_buffer[r];
                rot_index++;
                }
        }
        else if(found_R==false&&found_V==true){
            vel_index=0;
            for (int v=index_V+1;v<16;v++){
                vel_string[vel_index]=input_buffer[v];
                vel_index++;
                }
        }
        else{
            pc.printf("Input not recognised");
        }
        rotations_user=atof(rot_string);
        required_velocity=atof(vel_string);
        }
    }

void play(){
    if(note==A){
        frequency=440.0;
    }
     else if(note==B){
        frequency= 493.88;
    }
    else if(note==C){
        frequency=261.63;
    }
    else if(note==D){
        frequency=293.66;
    }
    else if(note==E){
        frequency=329.63;
    }
    else if(note==F){
        frequency=349.23;
    }
    else if(note==G){
        frequency=392.00;
    }
    else{
        frequency=1.0;
    }
    
}
