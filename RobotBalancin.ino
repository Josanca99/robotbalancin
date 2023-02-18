////////////////////////////
///CONFIGURACIÓN DE PINES///
////////////////////////////

//Pines MPU-6050//

//Vcc --> 3.3V
//GND --> GND
//SCL --> A5
//SDA --> A4
//INT --> D2

//Pines A4988 (Derecha)//

//DIR --> D3
//STEP --> D4
//n/SLEEP --> RESET
//n/RESET --> SLEEP
//MS3 --> 5V
//MS2 --> 5V
//MS1 --> 5V
//n/ENABLE --> Aire
//GND --> GND
//VDD --> 5V
//1B --> Azul Motor
//1A --> Rojo Motor
//2A --> Verde Motor
//2B --> Negro Motor
//GND --> GND
//VMOT --> 12V

//Pines A4988 (Izquierda)//

//DIR --> D5
//STEP --> D6
//n/SLEEP --> RESET
//n/RESET --> SLEEP
//MS3 --> 5V
//MS2 --> 5V
//MS1 --> 5V
//n/ENABLE --> Aire
//GND --> GND
//VDD --> 5V
//1B --> Azul Motor
//1A --> Rojo Motor
//2A --> Verde Motor
//2B --> Negro Motor
//GND --> GND
//VMOT --> 12V

//HC-05//
//RX --> D1/TX
//TX --> D0/RX
//GND --> GND
//+5V --> 5V

///////////////
///LIBRERÍAS///
///////////////

#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro

int MPU_direccion = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int MPU_calibracion = -8257;                       //Enter the accelerometer calibration value

//Various settings
float Kp_ganancia = 25 ;                                       //Gain setting for the P-controller (15)
float Ki_ganancia = 1;                                      //Gain setting for the I-controller (1.5)
float Kd_ganancia = 5;                                       //Gain setting for the D-controller (30)
float velocidad_giro = 30;                                    //Turning speed (20)
float velocidad_objetivo_max = 150;                                //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, byte_recibido, low_bat;

int motor_izq, aceleracion_mi, cont_aceleracion_mi, mem_aceleracion_mi;
int motor_der, aceleracion_md, cont_aceleracion_md, mem_aceleracion_md;
int battery_voltage;
int contador_recibido;
int giroscopio_pitch, giroscopio_yaw, acelerometro_dato;

long giroscopio_yaw_cal, giroscopio_pitch_cal;

unsigned long loop_timer;

float ang_giro, ang_ace, angle, pid_setpoint_robot;
float pid_error, pid_memoria, pid_setpoint, giro_input, pid_salida, pid_ultimo_error;
float pid_salida_izq, pid_salida_der;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
  
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(MPU_direccion);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(MPU_direccion);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(MPU_direccion);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(MPU_direccion);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 

  pinMode(3, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(6, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output

  for(contador_recibido = 0; contador_recibido < 500; contador_recibido++){       //Create 500 loops
    if(contador_recibido % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(MPU_direccion);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(MPU_direccion, 4);                                      //Request 2 bytes from the gyro
    giroscopio_yaw_cal += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    giroscopio_pitch_cal += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  giroscopio_yaw_cal /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  giroscopio_pitch_cal /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if(Serial.available()){                                                   //If there is serial data available
    byte_recibido = Serial.read();                                          //Load the received serial data in the received_byte variable
    contador_recibido = 0;                                                    //Reset the receive_counter variable
  }
  if(contador_recibido <= 25)contador_recibido ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else byte_recibido = 0x00;                                                //After 100 milliseconds the received byte is deleted
  //Load the battery voltage to the battery_voltage variable.
  //85 is the voltage compensation for the diode.
  //Resistor voltage divider => (3.3k + 3.3k)/2.2k = 2.5
  //12.5V equals ~5V @ Analog 0.
  //12.5V equals 1023 analogRead(0).
  //1250 / 1023 = 1.222.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  //battery_voltage = (analogRead(0) * 1.222) + 85;
  
  //if(battery_voltage < 1050 && battery_voltage > 800){                      //If batteryvoltage is below 10.5V and higher than 8.0V
  //  digitalWrite(13, HIGH);                                                 //Turn on the led if battery voltage is to low
  // low_bat = 1;                                                            //Set the low_bat variable to 1
 // }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  Wire.beginTransmission(MPU_direccion);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(MPU_direccion, 2);                                        //Request 2 bytes from the gyro
  acelerometro_dato = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  acelerometro_dato += MPU_calibracion;                          //Add the accelerometer calibration value
  if(acelerometro_dato > 8200)acelerometro_dato = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(acelerometro_dato < -8200)acelerometro_dato = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

  ang_ace = asin((float)acelerometro_dato/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && ang_ace > -0.5&& ang_ace < 0.5){                     //If the accelerometer angle is almost 0
    ang_giro = ang_ace;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  Wire.beginTransmission(MPU_direccion);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(MPU_direccion, 4);                                        //Request 4 bytes from the gyro
  giroscopio_yaw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  giroscopio_pitch = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  
  giroscopio_pitch -= giroscopio_pitch_cal;                      //Add the gyro calibration value
  ang_giro += giroscopio_pitch * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  giroscopio_yaw -= giroscopio_yaw_cal;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  ang_giro -= giroscopio_yaw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  ang_giro = ang_giro * 0.9996 + ang_ace * 0.0004 ;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error = ang_giro - pid_setpoint_robot - pid_setpoint;
  if(pid_salida > 10 || pid_salida < -10)pid_error += pid_salida * 0.015 ;

  pid_memoria += Ki_ganancia * pid_error;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_memoria > 400)pid_memoria = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_memoria < -400)pid_memoria = -400;
  //Calculate the PID output value
  pid_salida = Kp_ganancia * pid_error + pid_memoria + Kd_ganancia * (pid_error - pid_ultimo_error);
  if(pid_salida > 400)pid_salida = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_salida < -400)pid_salida = -400;

  pid_ultimo_error = pid_error;                                        //Store the error for the next loop

  if(pid_salida < 10 && pid_salida > -10)pid_salida = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(ang_giro > 30 || ang_giro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_salida = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_memoria = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    pid_setpoint_robot = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_salida_izq = pid_salida;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_salida_der = pid_salida;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(byte_recibido & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_salida_izq += velocidad_giro;                                       //Increase the left motor speed
    pid_salida_der -= velocidad_giro;                                      //Decrease the right motor speed
  }
  if(byte_recibido & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_salida_izq -= velocidad_giro;                                       //Decrease the left motor speed
    pid_salida_der += velocidad_giro;                                      //Increase the right motor speed
  }

  if(byte_recibido & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_salida > velocidad_objetivo_max * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(byte_recibido & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_salida < velocidad_objetivo_max)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(byte_recibido & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_salida < 0)pid_setpoint_robot += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_salida > 0)pid_setpoint_robot -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_salida_izq > 0)pid_salida_izq = 405 - (1/(pid_salida_izq + 9)) * 5500;
  else if(pid_salida_izq < 0)pid_salida_izq = -405 - (1/(pid_salida_izq - 9)) * 5500;

  if(pid_salida_der > 0)pid_salida_der = 405 - (1/(pid_salida_der + 9)) * 5500;
  else if(pid_salida_der < 0)pid_salida_der = -405 - (1/(pid_salida_der - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_salida_izq > 0)motor_izq = 400 - pid_salida_izq;
  else if(pid_salida_izq < 0)motor_izq = -400 - pid_salida_izq;
  else motor_izq = 0;

  if(pid_salida_der > 0)motor_der = 400 - pid_salida_der;
  else if(pid_salida_der < 0)motor_der = -400 - pid_salida_der;
  else motor_der = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  aceleracion_mi = motor_izq;
  aceleracion_md = motor_der;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  cont_aceleracion_mi ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(cont_aceleracion_mi > mem_aceleracion_mi){             //If the number of loops is larger then the throttle_left_motor_memory variable
    cont_aceleracion_mi = 0;                                        //Reset the throttle_counter_left_motor variable
    mem_aceleracion_mi = aceleracion_mi;                       //Load the next throttle_left_motor variable
    if(mem_aceleracion_mi < 0){                                     //If the throttle_left_motor_memory is negative
      PORTD &= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      mem_aceleracion_mi *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD |= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(cont_aceleracion_mi == 1)PORTD |= 0b01000000;             //Set output 6 high to create a pulse for the stepper controller
  else if(cont_aceleracion_mi == 2)PORTD &= 0b10111111;             //Set output 6 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  cont_aceleracion_md ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(cont_aceleracion_md > mem_aceleracion_md){           //If the number of loops is larger then the throttle_right_motor_memory variable
    cont_aceleracion_md = 0;                                       //Reset the throttle_counter_right_motor variable
    mem_aceleracion_md = aceleracion_md;                     //Load the next throttle_right_motor variable
    if(mem_aceleracion_md < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD |= 0b11110111;                                                   //Set output 3 low to reverse the direction of the stepper controller
      mem_aceleracion_md *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if(cont_aceleracion_md == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
  else if(cont_aceleracion_md == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}
