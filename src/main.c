
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
#define INPUT_BUFFER_SIZE 256
#define OUTPUT_BUFFER_SIZE 256

enum state { IDLE=1, READ_SENSOR, SEND_MESSAGE, RECEIVE_MESSAGE, PROCESS_MESSAGE};
enum state currentState = IDLE;

char tx_message[OUTPUT_BUFFER_SIZE];
char rx_message[INPUT_BUFFER_SIZE];

uint8_t message_len = 0;
uint8_t consecutive_spaces = 0;

static void sensor_task(void *arg){
    (void)arg;
    // init

    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);*/
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }

    enum sensor_read {NO_MOTION, MOTION_HAPPENED};
    enum sensor_read imu_sensor_motion = NO_MOTION;

    while(1){
        
        //sensor task jutut
        if (currentState == IDLE) {
            currentState = READ_SENSOR;
            
            float ax, ay, az, gx, gy, gz, t;

            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            
                // printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                if (ax != -4.0f) {

                    if (abs(gx) < 150.0f && abs(gy) < 150.0f && abs(gz) < 150.0f) {

                        if (ax < -1.0f) {
                            imu_sensor_motion = MOTION_HAPPENED;
                            printf(".\n");
                            printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                        } else if (ay > 0.0f) {
                            imu_sensor_motion = MOTION_HAPPENED;
                            printf("-\n");
                            printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                        }else if (az > 1.0f) {
                            imu_sensor_motion = MOTION_HAPPENED;
                            printf(" (space)\n");
                            printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                        }

                    }

                }


            } else {
                printf("Failed to read imu data\n");
            }

            currentState = IDLE;
        }

        // Do not remove this
        if (imu_sensor_motion == NO_MOTION) {
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            imu_sensor_motion = NO_MOTION;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

static void send_task(void *arg){
    (void)arg;
    
    while(1){
        /*
        if (button_pressed) {
             for (int i = 0; hellotext[i] != NULL; i++) {
                printf("%s", hellotext[i]);
            }
            button_pressed = false;
        }
        if (debug_pressed){
            for (int i = 0; hellotext_debug[i] != NULL; i++) {
                printf("%s", hellotext_debug[i]);
            }
            debug_pressed = false;
        }
        */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void receive_task(void *arg){
    (void)arg;
    char line[INPUT_BUFFER_SIZE];
    size_t index = 0;
    
    while (1){
        //OPTION 1
        // Using getchar_timeout_us https://www.raspberrypi.com/documentation/pico-sdk/runtime.html#group_pico_stdio_1ga5d24f1a711eba3e0084b6310f6478c1a
        // take one char per time and store it in line array, until reeceived the \n
        // The application should instead play a sound, or blink a LED. 

        /*
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT){// I have received a character
            if (c == '\r') continue; // ignore CR, wait for LF if (ch == '\n') { line[len] = '\0';
            if (c == '\n'){
                // terminate and process the collected line
                line[index] = '\0'; 
                printf("__[RX]:\"%s\"__\n", line); //Print as debug in the output
                index = 0;
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
            }
            else if(index < INPUT_BUFFER_SIZE - 1){
                line[index++] = (char)c;
            }
            else { //Overflow: print and restart the buffer with the new character. 
                line[INPUT_BUFFER_SIZE - 1] = '\0';
                printf("__[RX]:\"%s\"__\n", line);
                index = 0; 
                line[index++] = (char)c; 
            }
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
        }*/

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for new message


        //OPTION 2. Use the whole buffer. 
        /*absolute_time_t next = delayed_by_us(get_absolute_time,500);//Wait 500 us
        int read = stdio_get_until(line,INPUT_BUFFER_SIZE,next);
        if (read == PICO_ERROR_TIMEOUT){
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
        }
        else {
            line[read] = '\0'; //Last character is 0
            printf("__[RX] \"%s\"\n__", line);
            vTaskDelay(pdMS_TO_TICKS(50));
        }*/
    }

}

// Handling display update
void process_task(void *pvParameters) {

    while (1) {
    
        if (currentState == PROCESS_MESSAGE) {
        
            // Functionality of state
            //update_screen();
            
            // State transition UPDATE -> IDLE
            currentState = IDLE;
        }
    
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


int main() {

    stdio_init_all();

    // Wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    
    TaskHandle_t hSensorTask, hSendTask, hReceiveTask, hProcessTask = NULL;


    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hSensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }

    result = xTaskCreate(send_task,  // (en) Task function
                "send",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hSendTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Send Task creation failed\n");
        return 0;
    }
    
    result = xTaskCreate(receive_task,  // (en) Task function
                "receive",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hReceiveTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Receive Task creation failed\n");
        return 0;
    }
    
    result = xTaskCreate(process_task,  // (en) Task function
                "process",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hProcessTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Process Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

