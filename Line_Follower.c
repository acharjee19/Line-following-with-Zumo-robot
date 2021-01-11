/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>

#if 1
    
//Group 6 - Line Follower    

struct sensors_ dig;  
    
#define BLACK 1
#define WHITE 0
    
void run_to(int color)                              
{
    struct sensors_ dig;   
    bool run = true;
    while(run)
    {
        reflectance_digital(&dig);                      //Robot moves to the start 
        if(dig.l3 == color && dig.r3 == color){         //line and stops
            motor_forward(0,0);
            run = false;
        }
        else {
            motor_forward(250,10);
        }
    }   
}
void follow_line()
{
    TickType_t miss_time, line_time;
    motor_start();
    motor_forward(0,0);
    int i = 0;                              //Number of intersections the robot crosses
    while(1)
    {
        reflectance_digital(&dig);                  //Counts number of intersections
		if(dig.l3 == 1 && dig.r3 == 1) {            //the robot is crossing. When the 
			i++;                                    //number reaches 3, the final line,
            if(i == 3){                             //the robot stops
                break;
            }
            run_to(WHITE);  
        }
        else if(dig.l1 == 1 && dig.r1 == 1) {       //Robot moves at top speed, when
			motor_forward(250,10);                  //both center sensors see black
        }
        else if(dig.l1 == 0 && dig.r1 == 1) {       //Keeps the center sensors on
			motor_turn(100,10,10);                  //the line 
        }
        else if(dig.l1 == 1 && dig.r1 == 0) {       
            motor_turn(10,100,10); 
        } 
        else if(dig.l3 == 1 && dig.r1 == 0) {       //When one of the outer sensors
			SetMotors(1,0,100,100,10);              //detects black, robot makes a
        }                                           //sharp tank turn to that side
        else if(dig.l1 == 0 && dig.r3 == 1) {
			SetMotors(0,1,100,100,10);
        }
        if(dig.l1 == 0 && dig.r1 == 0) {                //Takes timestamp if both 
            miss_time = xTaskGetTickCount();            //center sensors get off
            print_mqtt("Zumo6/miss","%d",miss_time);    //the line
            while(1){
                reflectance_digital(&dig);              //Waits until both center           
		        if(dig.l1 == 1 && dig.r1 == 1){         //sensors are back on the
                    break;                              //line and takes timestamp
                }
            }                               
            line_time = xTaskGetTickCount();            
            print_mqtt("Zumo6/line","%d",line_time);    
			motor_forward(250,10);                  
        }
	}   
}
void zmain(void)
{
    vTaskDelay(100);
    while(SW1_Read()) vTaskDelay(100);
    TickType_t start_time, stop_time, run_time;
    reflectance_start();
    reflectance_set_threshold(14000, 14000, 14000, 14000, 14000, 14000); // set center sensor threshold between 4000 and 24000
    motor_start();
    motor_forward(0,0);
    run_to(BLACK);
    print_mqtt("Zumo6/ready","line");
    
    IR_Start();
    IR_flush(); // clear IR receive buffer
    IR_wait();  // wait for IR command
    
    start_time = xTaskGetTickCount(); 
    print_mqtt("Zumo6/start","%d",start_time);

    follow_line();
    
    motor_forward(0,0);
    stop_time = xTaskGetTickCount(); 
    print_mqtt("Zumo6/stop","%d",stop_time);
    run_time = stop_time - start_time;
    print_mqtt("Zumo6/time","%d",run_time);

    while(true){
        vTaskDelay(100);
    }
}  
#endif
/* [] END OF FILE */
