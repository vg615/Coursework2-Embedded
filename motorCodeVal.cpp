#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"

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


int8_t orState;   
//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//create data structure of message_t
typedef struct{ 
uint8_t code;
uint32_t data; 
} message_t ;

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Create global instances (objects) of the classes required
Thread commOutT(osPriorityNormal,1024);
Thread decoder(osPriorityNormal,1024);
Thread motorCtrlT (osPriorityNormal,1024);

Mail<message_t,16> outMessages;
Queue<void, 8> inCharQ;
volatile uint64_t newKey=0;
volatile uint64_t newRotation=0;
volatile uint64_t newVelocity=256;

Mutex newKey_mutex;
Mutex newRotation_mutex;
Mutex newVelocity_mutex;



//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00}; //array. Indexing is made via driveState[i] 

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  

//Status LED
DigitalOut myled(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

uint32_t motorPower = 1000;

//Set a given drive state
void motorOut(int8_t driveState, uint32_t motorPower){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07]; // & is bitwise AND

    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(motorPower);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(motorPower);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(motorPower);
    if (driveOut & 0x20) L3H = 0;
    }
    
    
    
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    motorOut(0,1000); //start with max pwm
    wait(2.0);
    //Get the rotor state
    return readRotorState();
}



int32_t motorPosition;
void motorISR() {
 static int8_t oldRotorState;
 int8_t rotorState = readRotorState();
 motorOut((rotorState-orState+lead+6)%6, motorPower);
 if (rotorState - oldRotorState == 5) motorPosition--;
 else if (rotorState - oldRotorState == -5) motorPosition++;
 else motorPosition += (rotorState - oldRotorState);
 oldRotorState = rotorState;
 }

   
void commOutFn() {
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        switch(pMessage->code){
            case 1:
            pc.printf("Hash rate is %d \r\n", pMessage->data); // better than just print code and data
            break;
            case 2:
            pc.printf("Valid nonce is %0x16 \r\n", pMessage->data);
            break;
            case 3:
            pc.printf("Motor position is %d \r\n", pMessage->data);
            break;
            case 4:
            pc.printf("Velocity is %d \r\n", pMessage->data);
            break;          
            }
        outMessages.free(pMessage);
        }    
    }



void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc(); 
    pMessage->code = code;
    pMessage->data = data; 
    outMessages.put(pMessage);
}

void serialISR(){
uint8_t newChar = pc.getc();
inCharQ.put((void*)newChar);
}

void decode_thread(){
    pc.attach(&serialISR); //whenever we type on keyboard, read character
    uint8_t newChar;
    char command[32];//initialise the array to hold the characters of the command
    int i = 0; //initialise the index counter to iterate through the command array
        while(1) {
            osEvent newEvent = inCharQ.get();
            newChar = (uint8_t)newEvent.value.p;
            if(i<18){
                if(newChar == '\r' || newChar == '\n'){ //if the final character is 'r' it is a carriage return so the user has finished typing the command
                    command[i] = '\0'; 
                    i = 0; //reset the counter to be ready for the next command 
                    if(command[0] == 'R'){
                        //decode rotation command
                        newRotation_mutex.lock();
                        sscanf(command, "R%x", &newRotation);
                        newRotation_mutex.unlock();                        
                        }
                    if(command[0] == 'r'){
                        //decode rotation command
                        newRotation_mutex.lock();
                        sscanf(command, "r%x", &newRotation);
                        newRotation_mutex.unlock();                        
                        }                        
                    if(command[0] == 'V'){
                        //decode maximum speed
                        newVelocity_mutex.lock();
                        sscanf(command, "V%x", &newVelocity);
                        newVelocity_mutex.unlock();                         
                        }
                    if(command[0] == 'v'){
                        //decode maximum speed
                        newVelocity_mutex.lock();
                        sscanf(command, "v%x", &newVelocity);
                        newVelocity_mutex.unlock();                         
                        }                        
                    if(command[0] == 'K'){
                        //decode bitcoin key
                        newKey_mutex.lock();
                        sscanf(command, "K%x", &newKey);
                        newKey_mutex.unlock();
                    }
                    if(command[0] == 'k'){
                        //decode bitcoin key
                        newKey_mutex.lock();
                        sscanf(command, "k%x", &newKey);
                        newKey_mutex.unlock();
                    }                    
                }
                else{
                command[i] = newChar; //add new character to array as part fot eh current command being typed by user
                i++; 
                }
        }
        else{ //incoming command is larger than the buffer size (18) so do not put the character and reset the counter to zero
            i = 0; 
        }
    }
}

float sign(float x){
    if(x<0){
        return -1;
            }
    else{
        return 1;
        }
    }


  

void motorCtrlTick(){
 motorCtrlT.signal_set(0x1);
 }

void motorCtrlFn(){
 int32_t y_speed, y_rate;
 int32_t y_final;
 int32_t Velocity=0;
 int8_t iter_velocity = 0;
 int32_t current_position;
 int32_t prev_position=0;  //To compute velocity
 int32_t my_speed;
 float prev_error = 0.0f;
 float error;
 float delta_error;
 Ticker motorCtrlTicker;
 motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    while(1){
        motorCtrlT.signal_wait(0x1);
        current_position = motorPosition;
        Velocity = (current_position - prev_position)*10;
        prev_position = current_position;
        iter_velocity +=1;
        if(iter_velocity == 10){
            iter_velocity = 0;
            putMessage (3, current_position);
            putMessage (4, Velocity);
            }
        error = newRotation - (current_position/6.0f);
        delta_error = error - prev_error;
        prev_error = error;
        my_speed = newVelocity;
        if (my_speed == 0){
            y_speed = 1000;
                }
        else{
            y_speed = (int32_t) 20*(6*my_speed - abs(Velocity))*sign(error);  //extra processing in ISR
            }
        y_rate = int32_t (22*error + 22*delta_error);
        if (((Velocity < 0) && (y_speed > y_rate)) || ((Velocity >= 0) && (y_speed < y_rate))) {
            y_final = y_speed;
        } else {
            y_final = y_rate;
        }        
        
        if (y_final > 0) {
            lead = 2;
        } else {
            y_final = -y_final;  
            lead = -2;         
        }
        if (y_final > 1000){
            y_final = 1000; // set max value
        }

        motorPower = y_final;    
        motorISR();
        }
    }
volatile uint32_t hash_counter=0;    
void printhashcounter(){
    putMessage(1, (uint32_t) (hash_counter));
    hash_counter = 0;
    }


//Main
int main() {
    
    //initialise PWM period
    L1L.period_us(2000);
    L2L.period_us(2000);
    L3L.period_us(2000);
    //Run the motor synchronisation
    orState = motorHome();
    
    commOutT.start(commOutFn);
    decoder.start(decode_thread);
    motorCtrlT.start(motorCtrlFn);
    

    
    I1.rise(&motorISR);  //attach the interrupts to the address of the "motorOutWithInterrupt" function
    I1.fall(&motorISR);
    I2.rise(&motorISR);
    I2.fall(&motorISR);
    I3.rise(&motorISR);
    I3.fall(&motorISR);
 
    
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];  
    SHA256 my_hash;
    uint64_t key_prev = *key; 
    Ticker t;
    t.attach(&printhashcounter, 1.0);    //when t = 1s, print hash rate and reset counter
    while(1){
    newKey_mutex.lock();
    *key = newKey;
    newKey_mutex.unlock();
    if(*key != key_prev){ //why dow e still enter this loop even if that same key is entered?
    key_prev = *key; 
    }  
    my_hash.computeHash(hash,sequence,64);
    hash_counter++;       
        if(hash[0] == 0 && hash[1] == 0){
            putMessage(2, *nonce);  
        }
    *nonce = *nonce + 1 ;    
    }
}

