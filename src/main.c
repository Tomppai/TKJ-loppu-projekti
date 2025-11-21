#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#include "images.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
#define INPUT_BUFFER_SIZE 256
#define OUTPUT_BUFFER_SIZE 256

// Motion constants
#define NO_MOTION_THRESHOLD 0.2f
#define MOTION_THRESHOLD 1.2f
#define OTHER_MOTION_THRESHOLD 0.6f
#define AY_OFFSET 0.2f
#define GYRO_THRESHOLD 200.0f

// Program states
enum state { IDLE=1, READ_SENSOR, SEND_MESSAGE, RECEIVE_MESSAGE, PROCESS_MESSAGE};
enum state currentState = IDLE;

// global tx_message for sending the message from pico to computer and rx_message to receive message from computer
char tx_message[OUTPUT_BUFFER_SIZE];
char rx_message[INPUT_BUFFER_SIZE];

// a task for reading the accelerometer and gyro to input dot/dash/space for a message
static void sensor_task(void *arg){

    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }

    // the states of the sensor
    enum sensor_read {NO_MOTION, MOTION, MOTION_HAPPENED};
    enum sensor_read imu_sensor_motion = MOTION_HAPPENED;

    // data about the message
    uint8_t message_len = 0;
    uint8_t consecutive_spaces = 0;
    
    // variables for the sensor data
    float ax, ay, az, gx, gy, gz, t;

    while(1){
        
        //sensor task jutut
        if (currentState == IDLE) {
            currentState = READ_SENSOR;
            
            // read sensor and put the data into sensor variables
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                // Next line is for analyzing the sensor data
                //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f\n", ax, ay, az, gx, gy, gz);
                
                // changes the sensor state to NO_MOTION if MOTION_HAPPENED and acceleration values are low enough
                if (imu_sensor_motion == MOTION_HAPPENED && fabs(ax) < NO_MOTION_THRESHOLD && fabs(ay+1.0f) < NO_MOTION_THRESHOLD && fabs(az) < NO_MOTION_THRESHOLD) {
                    imu_sensor_motion = NO_MOTION;
                }
            
                // checks that there is no motion and the device is not being rotated
                if ((imu_sensor_motion == NO_MOTION) && (fabs(gx)+ fabs(gy) + fabs(gz) < GYRO_THRESHOLD)) {

                    // dot
                    if (ax > MOTION_THRESHOLD && fabs(ay+1.0f) < OTHER_MOTION_THRESHOLD && fabs(az) < OTHER_MOTION_THRESHOLD) {
                        imu_sensor_motion = MOTION;

                        tx_message[message_len] = '.';
                        message_len++;
                        consecutive_spaces = 0;
                        //printf("dot\n");

                        // Sound for dot
                        buzzer_play_tone (440, 200);

                    }

                    // dash
                    else if (ay+1.0f > MOTION_THRESHOLD - AY_OFFSET && fabs(ax) < OTHER_MOTION_THRESHOLD && fabs(az) < OTHER_MOTION_THRESHOLD) {
                        imu_sensor_motion = MOTION;

                        tx_message[message_len] = '-';
                        message_len++;
                        consecutive_spaces = 0;
                        //printf("dash\n");

                        // Sound for dash
                        buzzer_play_tone (300, 500);

                    }
                    
                    // space
                    else if (az > MOTION_THRESHOLD && fabs(ay+1.0f) < OTHER_MOTION_THRESHOLD && fabs(ax) < OTHER_MOTION_THRESHOLD) {
                        imu_sensor_motion = MOTION;

                        tx_message[message_len] = ' ';
                        message_len++;
                        consecutive_spaces++;
                        //printf("space\n");

                        // Sound for space
                        buzzer_play_tone (350, 100);
                        buzzer_play_tone (850, 100);
                    }

                }


            } else {
                printf("Failed to read imu data\n");
            }
            
            // checks if message is ready to be sent
            if (message_len == 254 || consecutive_spaces == 3) {
                tx_message[message_len] = '\0';
                consecutive_spaces = 0;
                message_len = 0;
                
                currentState = SEND_MESSAGE;
            } 
            else {
                if (currentState == READ_SENSOR) {
                    currentState = IDLE;
                }
            }
        }

        if (imu_sensor_motion == MOTION) {
            imu_sensor_motion = MOTION_HAPPENED;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// sends the message to the computer
static void send_task(void *arg){

    while(1){

        if (currentState == SEND_MESSAGE) {
            for (int i = 0; tx_message[i] != '\0'; i++) {
                printf("%c", tx_message[i]);
            }
            printf("\n");

            clear_display();
            show_image(messagesent, messagesent_size);

            // plays the super mario bros. theme
            play_note(note_E, 4, 167);
            play_note(note_E, 4, 167);
            sleep_ms(167);
            play_note(note_E, 4, 167);
            sleep_ms(167);
            play_note(note_C, 4, 167);
            play_note(note_E, 4, 333);
            play_note(note_G, 4, 333);
            sleep_ms(333);
            play_note(note_G, 3, 333);

            show_image(tmlogo, tmlogo_size);
            currentState = IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// checks if a message has been received
static void receive_task(void *arg) {

    while (1){

        if (currentState == IDLE) {
            currentState = RECEIVE_MESSAGE;

            absolute_time_t next = delayed_by_us(get_absolute_time(), 500);//Wait 500 us
            int read = stdio_get_until(rx_message, INPUT_BUFFER_SIZE, next);
            if (read == PICO_ERROR_TIMEOUT){
                currentState = IDLE;
                vTaskDelay(pdMS_TO_TICKS(300)); // Wait for new message
            }
            else {
                rx_message[read] = '\0'; //Last character is 0
                printf("__[RX] \"%s\"__\n", rx_message);

                show_image(incomingcall, incomingcall_size);

                // plays nokia ringtone
                play_note(note_E, 6, 150);
                play_note(note_D, 6, 150);
                play_note(note_Fsharp, 5, 300);
                play_note(note_Gsharp, 5, 300);
                play_note(note_Csharp, 6, 150);
                play_note(note_B, 5, 150);
                play_note(note_D, 5, 300);
                play_note(note_E, 5, 300);
                play_note(note_B, 5, 150);
                play_note(note_A, 5, 150);
                play_note(note_Csharp, 5, 300);
                play_note(note_E, 5, 300);
                play_note(note_A, 5, 600);

                clear_display();

                currentState = PROCESS_MESSAGE;

                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
    }

}

// outputs the received message
void process_task(void *pvParameters) {

    while (1) {

        if (currentState == PROCESS_MESSAGE) {

            // for each character, draws the symbol on the screen and plays the corresponding sound
            for (int i = 0; rx_message[i] != '\0'; i++) {
                switch (rx_message[i])
                {
                case '.':
                    draw_circle(64, 32, 8, true);
                    buzzer_play_tone (440, 200);
                    break;

                case '-':
                    draw_square(32, 24, 64, 16, true);
                    buzzer_play_tone (300, 500);
                    break;

                case ' ':
                    buzzer_play_tone (350, 100);
                    buzzer_play_tone (850, 100);
                    break;

                // if the character is not in morse code, it will be displayed on the screen
                default:
                    char buf[2] = {rx_message[i], '\0'};
                    write_text(buf);
                    buzzer_play_tone (200, 100);
                    break;
                }

                clear_display();
                sleep_ms(130);
            }
            sleep_ms(500);

            // plays megalovania
            play_note(note_D, 4, 200);
            play_note(note_D, 4, 200);
            play_note(note_D, 5, 400);
            play_note(note_A, 4, 600);
            play_note(note_Gsharp, 4, 400);
            play_note(note_G, 4, 400);
            play_note(note_F, 4, 400);
            play_note(note_D, 4, 200);
            play_note(note_F, 4, 200);
            play_note(note_G, 4, 200);

            show_image(tmlogo, tmlogo_size);
            currentState = IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int main() {

    stdio_init_all();

    // Wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    
    init_hat_sdk();
    sleep_ms(400); //Wait some time so initialization of USB and hat is done.

    init_display();
    init_buzzer();
    sleep_ms(200); //Wait some time so initialization of display and buzzer is done.
    
    // main screen image
    show_image(tmlogo, tmlogo_size);

    // plays windows xp startup sound
    play_note(note_Dsharp, 5, 450);
    play_note(note_Dsharp, 4, 150);
    play_note(note_Asharp, 4, 300);
    play_note(note_Gsharp, 4, 300);
    play_note(note_Dsharp, 4, 300);
    play_note(note_Dsharp, 5, 300);
    play_note(note_Asharp, 4, 600);

    TaskHandle_t hSensorTask, hSendTask, hReceiveTask, hProcessTask = NULL;

    // Create the tasks with xTaskCreate

    // Sensor task
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

    // Send task
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
    
    // Receive task
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
    
    // Process task
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

    // all task initializations are done and passed
    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

