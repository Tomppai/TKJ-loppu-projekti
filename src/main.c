
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#include "incomingcall.h"
#include "messagesent.h"
#include "tmlogo.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
#define INPUT_BUFFER_SIZE 256
#define OUTPUT_BUFFER_SIZE 256

// Motion constants
#define NO_MOTION_THRESHOLD 0.2f
#define MOTION_THRESHOLD 1.0f
#define OTHER_MOTION_THRESHOLD 0.7f
#define AY_OFFSET 0.2f
#define GYRO_THRESHOLD 150.0f

void show_image(const uint8_t* data, const long size);

// Program states
enum state { IDLE=1, READ_SENSOR, SEND_MESSAGE, RECEIVE_MESSAGE, PROCESS_MESSAGE};
enum state currentState = IDLE;

enum note {C=1, Csharp, D, Dsharp, E, F, Fsharp, G, Gsharp, A, Asharp, B};

char tx_message[OUTPUT_BUFFER_SIZE];
char rx_message[INPUT_BUFFER_SIZE];

uint8_t message_len = 0;
uint8_t consecutive_spaces = 0;

void play_note(enum note cur_note, int octave, int duration);

static void sensor_task(void *arg){
    (void)arg;
    // init

    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        //printf("ICM-42670P initialized successfully!\n");
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

    init_buzzer();

    enum sensor_read {NO_MOTION, MOTION, MOTION_HAPPENED};
    enum sensor_read imu_sensor_motion = NO_MOTION;
    
    float ax, ay, az, gx, gy, gz, t;

    while(1){
        
        //sensor task jutut
        if (currentState == IDLE) {
            currentState = READ_SENSOR;
            

            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

                if (imu_sensor_motion == MOTION_HAPPENED && fabs(ax) < NO_MOTION_THRESHOLD && fabs(ay+1.0f) < NO_MOTION_THRESHOLD && fabs(az) < NO_MOTION_THRESHOLD) { // !(ax > 0.2f || ay > 0.8f ||  az > 0.2f)
                    imu_sensor_motion = NO_MOTION;
                }
            
                //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                if (ax != -4.0f && imu_sensor_motion == NO_MOTION) {

                    if (fabs(gx) < GYRO_THRESHOLD && fabs(gy) < GYRO_THRESHOLD && fabs(gz) < GYRO_THRESHOLD) {

                        if (ax < -1.0f * MOTION_THRESHOLD && fabs(ay+1.0f) < OTHER_MOTION_THRESHOLD && fabs(az) < OTHER_MOTION_THRESHOLD) {
                            imu_sensor_motion = MOTION;

                            tx_message[message_len] = '.';
                            message_len++;
                            consecutive_spaces = 0;

                            //printf(".\n");
                            //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);

                            // Sound for dot
                            buzzer_play_tone (440, 200);

                        } else if (ay+1 > MOTION_THRESHOLD - AY_OFFSET && fabs(ax) < OTHER_MOTION_THRESHOLD && fabs(az) < OTHER_MOTION_THRESHOLD) {
                            imu_sensor_motion = MOTION;

                            tx_message[message_len] = '-';
                            message_len++;
                            consecutive_spaces = 0;

                            //printf("-\n");
                            //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);

                            // Sound for dash
                            buzzer_play_tone (300, 500);

                        } else if (az > MOTION_THRESHOLD && fabs(ay+1.0f) < OTHER_MOTION_THRESHOLD && fabs(ax) < OTHER_MOTION_THRESHOLD) {
                            imu_sensor_motion = MOTION;

                            tx_message[message_len] = ' ';
                            message_len++;
                            consecutive_spaces++;

                            //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);

                            // Sound for space
                            buzzer_play_tone (350, 100);
                            buzzer_play_tone (850, 100);
                        }

                    }

                }


            } else {
                printf("Failed to read imu data\n");
            }
            
            if (message_len == 254 || consecutive_spaces == 3) {
                tx_message[message_len+1] = '\0';
                consecutive_spaces = 0;
                message_len = 0;
                
                currentState = SEND_MESSAGE;
            } else {
                if (currentState == READ_SENSOR) {
                    currentState = IDLE;
                }
            }
        }

        // Do not remove this
        if (imu_sensor_motion != MOTION) {
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            imu_sensor_motion = MOTION_HAPPENED;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static void send_task(void *arg){
    (void)arg;
    
    while(1){
        
        if (currentState == SEND_MESSAGE) {
            for (int i = 0; tx_message[i] != '\0'; i++) {
                printf("%c", tx_message[i]);
            }
            printf("\n");

            show_image(messagesent, messagesent_size);
            // plays the super mario bros. theme
            play_note(E, 4, 167);
            playnote(E, 4, 167);
            wait_ms(167);
            play_note(E, 4, 167);
            wait_ms(167);
            play_note(C, 4, 167);
            play_note(E, 4, 333);
            play_note(G, 4, 333);
            wait_ms(333);
            play_note(G, 3, 333);

            show_image(tmlogo, tmlogo_size);
            currentState = IDLE;
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void receive_task(void *arg) {
    (void)arg;
    size_t index = 0;
    
    while (1){

        // Use the whole buffer. 
        if (currentState == IDLE) {

            absolute_time_t next = delayed_by_us(get_absolute_time(), 500);//Wait 500 us
            int read = stdio_get_until(rx_message, INPUT_BUFFER_SIZE, next);
            if (read == PICO_ERROR_TIMEOUT){
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
            }
            else {
                rx_message[read] = '\0'; //Last character is 0
                printf("__[RX] \"%s\"__\n", rx_message);

                //write_text("Tomp call");
                show_image(incomingcall, incomingcall_size);

                // plays nokia ringtone
                play_note(E, 6, 150);
                play_note(D, 6, 150);
                play_note(Fsharp, 5, 300);
                play_note(Gsharp, 5, 300);
                play_note(Csharp, 6, 150);
                play_note(B, 5, 150);
                play_note(D, 5, 300);
                play_note(E, 5, 300);
                play_note(B, 5, 150);
                play_note(A, 5, 150);
                play_note(Csharp, 5, 300);
                play_note(E, 5, 300);
                play_note(A, 5, 600);
    
                currentState = PROCESS_MESSAGE;
                clear_display();
                
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }

}

// Handling display update
void process_task(void *pvParameters) {

    while (1) {
    
        if (currentState == PROCESS_MESSAGE) {
        
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
                
                default:
                    char buf[2] = {rx_message[i], '\0'}; //Store a number of maximum 5 figures 
                    // sprintf(buf, "%c", rx_message[i]);
                    write_text(buf);
                    buzzer_play_tone (200, 100);
                    break;
                }

                clear_display();
                sleep_ms(130);
            }

            // plays megalovania
            play_note(D, 4, 200);
            play_note(D, 4, 200);
            play_note(D, 5, 400);
            play_note(A, 4, 600);
            play_note(Gsharp, 4, 400);
            play_note(G, 4, 400);
            play_note(F, 4, 400);
            play_note(D, 4, 200);
            play_note(F, 4, 200);
            play_note(G, 4, 200);
            
            show_image(tmlogo, tmlogo_size);
            currentState = IDLE;
        }
    
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void show_image(const uint8_t* data, const long size) {
    clear_display();
    ssd1306_bmp_show_image(&disp, data, size);
}

void play_note(enum note cur_note, int octave, int duration) {
    float frequency = 0.0f;
    switch (cur_note)
    {
    
    case C:
        frequency = 16.35f;
        break;

    case Csharp:
        frequency = 17.32f;
        break;
    
    case D:
        frequency = 18.35f;
        break;

    case Dsharp:
        frequency = 19.45f;
        break;
    
    case E:
        frequency = 20.60f;
        break;
    
    case F:
        frequency = 21.83f;
        break;

    case Fsharp:
        frequency = 23.12f;
        break;
    
    case G:
        frequency = 24.50f;
        break;

    case Gsharp:
        frequency = 25.96f;
        break;
    
    case A:
        frequency = 27.50f;
        break;

    case Asharp:
        frequency = 29.14f;
        break;
    
    case B:
        frequency = 30.87f;
        break;
    
    default:
        break;
    }

    frequency *= (float) pow(2, octave);

    buzzer_play_tone (frequency, duration);
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
    sleep_ms(400); //Wait some time so initialization of USB and hat is done.
    clear_display();
    show_image(tmlogo, tmlogo_size);

    // plays windows xp startup sound
    play_note(Dsharp, 5, 450);
    play_note(Dsharp, 4, 150);
    play_note(Asharp, 4, 300);
    play_note(Gsharp, 4, 300);
    play_note(Dsharp, 4, 300);
    play_note(Dsharp, 5, 300);
    play_note(Asharp, 4, 600);


    
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

