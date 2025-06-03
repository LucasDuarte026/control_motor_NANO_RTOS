#include <Arduino.h>
#include <time.h>        // For std::chrono literals (e.g., 1s, 100ms) for sleep_for
#include <chrono>        // For std::chrono literals (e.g., 1s, 100ms) for sleep_for
#include "Encoder.h"     // Paul Stoffregen's Encoder library
#include "mbed.h" // Inclua o cabeçalho do Mbed OS


using namespace rtos;
using namespace mbed;       // For ThisThread, Callback, etc.

// Define the pins connected to the encoder A and B phases
const int ENCODER_PIN_A = 8; // Example pin D8
const int ENCODER_PIN_B = 9; // Example pin D9
Encoder myEncoder(ENCODER_PIN_A, ENCODER_PIN_B);

Queue<long, 10> encoder_data_queue;

// --- Task Function Prototypes ---
void system_health_task();    // Your LED blinking task
void encoder_reader_task();   // Task to read encoder and put data in queue
void encoder_printer_task();  // Task to get data from queue and print it

// --- Task Implementations ---

void system_health_task() {
    for (;;) {

        digitalWrite(LED_BUILTIN, HIGH); // Turn LED on er
        Serial.print("System health task: LED ON time is: ");
        Serial.print(time(NULL)); Serial.println();
        ThisThread::sleep_for(std::chrono::seconds(1)); // Use chrono duration
        digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
        ThisThread::sleep_for(std::chrono::seconds(1)); // Use chrono duration
    }
}

void encoder_reader_task() {
    long currentPosition = 0;
    long lastPosition = -999; // Initialize to ensure first read is different

    for (;;) {
        currentPosition = myEncoder.read(); // Read the current encoder position

        // Only send data if the position has changed
        if (currentPosition != lastPosition) {
            // Try to send the new position to the queue without blocking.
            // Returns true if sent, false if queue is full.
            if (encoder_data_queue.try_put(&currentPosition)) {
                 // Data successfully sent.
            } else {
                // If the queue is full, it means the printer task is not keeping up.
                Serial.println("Encoder queue full, data lost!");
            }
            lastPosition = currentPosition; // Update last position
        }

        // Add a small delay to allow other tasks to run.
        ThisThread::sleep_for(std::chrono::milliseconds(1)); // Check every 1ms
    }
}

void encoder_printer_task() {
    long receivedPosition;

    for (;;) {
        // Get an osEvent from the queue. This call blocks indefinitely (osWaitForever)
        // until an item is available.
        osEvent evt = encoder_data_queue.get(osWaitForever);
        Serial.print("System health task: LED ON time is: ");
        Serial.print(time(NULL)); Serial.println();
        // Check the status of the event.
        // osEventMessage indicates that a message (data) was successfully received.
        if (evt.status == osEventMessage) {
            // The received data is in evt.value.p (a pointer to void).
            // Cast it back to a pointer of the correct type (long*) and dereference it.
            receivedPosition = *(long*)evt.value.p;
            Serial.print("Encoder Position: ");
            Serial.println(receivedPosition);
        } else {
            // This case should ideally not be reached with osWaitForever,
            // unless there's a critical error in Mbed OS (e.g., queue deleted).
            Serial.print("Error receiving from encoder queue! Status: ");
            Serial.println(evt.status); // Print status for debugging
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT); // Configure the built-in LED as output

    // Give some time for the serial monitor to connect
    ThisThread::sleep_for(std::chrono::milliseconds(2000)); // 2 seconds
    Serial.println("Arduino Nano RP2040 Connect: Starting Mbed OS (FreeRTOS) tasks with encoder...");

    // Create and start tasks
    // Use `new Thread(...)` and `->start(callback(...))`
    Thread *sys_health_thread = new Thread(osPriorityNormal);
    sys_health_thread->start(callback(system_health_task));

    Thread *encoder_read_thread = new Thread(osPriorityNormal); // High priority for reading
    encoder_read_thread->start(callback(encoder_reader_task));

    Thread *encoder_print_thread = new Thread(osPriorityNormal); // Lower priority for printing
    encoder_print_thread->start(callback(encoder_printer_task));

    // The scheduler takes over from here. loop() will not be called.
}
void loop() {}
