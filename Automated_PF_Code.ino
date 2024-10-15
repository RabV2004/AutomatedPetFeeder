#include <stdio.h> // Standard I/O library for input/output functions

// Initialize count, timer, and other variables
volatile unsigned long timeCounter = 0; // to avoid overflow issues
float currentCondition = 0;      // Holds a float value for tracking any condition, not used currently
int actionFlag = 0;              // Flag variable to signal some event or action
unsigned int catInterval = 40;   // Interval (time) for feeding the cat
unsigned int dogInterval = 100;  // Interval (time) for feeding the dog
static FILE uart00 = {0};        // Defines a static FILE variable for UART communication
unsigned int sum = 0;            // Accumulates values for averaging the weight sensor readings
unsigned int mean;               // Stores the calculated mean of sensor values
static int weightSensor();       // Function prototype for reading and averaging values from the weight sensor
int sensorValue;                 // Stores the value returned from the weightSensor function
void playBuzzer(int tune[], int duration[], int numNotes); // Prototype for buzzer-playing function
void Catbuzzer();               // Prototype for cat-specific buzzer sound
void Dogbuzzer();               // Prototype for dog-specific buzzer sound

// Initializes USART0 for serial communication
void USART0_init(void) {
    // UCSR0A: Normal speed mode, no multi-processor mode
    UCSR0A = 0B00000000;

    // UCSR0B: Enable transmitter (TX) and receiver (RX)
    UCSR0B = 0B00011000;

    // UCSR0C: Asynchronous mode, 8 data bits, no parity, 1 stop bit
    UCSR0C = 0B00000110;

    // Set baud rate to 9600 (assuming 16 MHz clock)
    UBRR0 = 103;

    // Configure standard I/O streams to use USART for TX/RX
    fdev_setup_stream(&uart00, TX, RX, _FDEV_SETUP_RW);
    stdin = stdout = &uart00;  // Redirect stdin and stdout to USART
}

// Function for receiving data over USART0
static int RX(FILE *stream) {
    while ((UCSR0A & (1 << RXC0)) == 0) {}; // Wait until data is received (RX complete flag is set)
    return(UDR0); // Return the received data from the UDR0 register
}

// Function for transmitting data over USART0
static int TX(char TXData, FILE *stream) {
    while ((UCSR0A & (1 << UDRE0)) == 0) {}; // Wait until UDR0 is ready for new data (Data Register Empty flag is set)
    UDR0 = TXData; // Load the data to be transmitted into the UDR0 register
    return 0;
}

/* Processes load cell data using the microcontroller’s ADC by averaging 20 readings to reduce noise - Basic DC Smoothing
   the weightSensor() function reads the ADC values 20 times in this case and takes the mean of it for a more credible value to account for noise and variations. 
   The function is "static" to limit its scope to this file, preventing accidental use elsewhere : encapsulation */
static int weightSensor() {
    int numReadings = 20;  // Define the number of sensor readings to take for averaging
    int sum = 0;  // Initialize sum to accumulate ADC values

    for (int i = 0; i < numReadings; i++) {  // Loop to take multiple readings
        ADMUX = 0B11001011;  // Select ADC input channel and reference voltage
        ADCSRB = 0B00000000; // Configure ADC features
        ADCSRA = 0B11010111; // Enable ADC and start conversion

        _delay_ms(1);  // Short delay for a clearer sensor result
        while (!(ADCSRA & (1 << ADIF)));  // Wait until ADC conversion completes
        ADCSRA |= (1 << ADIF);  // Clear the ADC interrupt flag

        sum += ADC;  // Accumulate the ADC value
        _delay_ms(50);  // Delay between each sensor reading
    }

    int mean = sum / numReadings;  // Calculate the mean of the sensor readings
    _delay_ms(100);  // More delay for better observation
    return mean;  // Return the calculated mean value
}

// Servo motor control via PWM (direct register access). 
// determines the servo position by adjusting the pulse width.
void setServoPosition(int position) {
    OCR3B = position;  // Set the Output Compare Register 3B (OCR3B) to control the PWM duty cycle.
}

// ISR for Timer 1 (manages time counting)
ISR(TIMER1_COMPC_vect) {
    TCNT1 = 0xC2F7;  // Reload the Timer 1 counter with a preloaded value (49911 in decimal). Ensures that the timer starts counting from this point on each overflow.
    timeCounter++;   // Increment the time counter to keep track of elapsed time. Used to trigger periodic events like feeding intervals.
}

/* Using Fast PWM mode with Timer 3 to control the servos. 
    In the setup() function, I configure Timer 3 using the TCCR3A and TCCR3B registers to run in mode 14 (Fast PWM with ICR3 as the top value). 
    By setting different values in OCR3B, we control the duty cycle, which changes the servo position. 
    The 50% duty cycle (set in OCR3B) opens the servo, and a lower duty cycle closes it. 
    The prescaler is set to clkIO/256 to slow down the PWM signal enough to work for the servo. */
    
// System setup, runs once during initialization
void setup() {
    USART0_init();  // Initialize USART0 for serial communication

    // Configure Timer 3 for Fast PWM mode
    DDRE = (1<<DDE3)|(1<<DDE4); // Set pins PE3 and PE4 (Arduino pins 5 and 2) as output pins (OC3A and OC3B)
    TCCR3A = (1<<COM3A1)| (1<<COM3B1)|(1<<WGM31); // Configure Fast PWM mode 14, output compare for channel A and B
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS32); // Fast PWM mode 14, set prescaler to clkIO/256
    
    DDRB = (1 << DDB7);   // Set pin 13 (Arduino pin) as output
    cli(); // Disable global interrupts to set up timers and other critical settings
    
    // Configure Timer 1 to manage timing and interrupts
    TCCR1A = (1 << COM1C0); // Toggle OC1C on compare match
    TCCR1B = (1 << CS12) | (1 << CS10); // Set prescaler to clk/1024 for Timer 1 (slower timer)
    TCCR1C = 0; // Clear Timer/Counter Control Register C
    OCR1C = 0xFFFF; // Set the compare value for Timer 1 (max count value)
    TCNT1 = 0xC2F7; // Initialize Timer 1 counter to 49911 (preloaded value for timing)

    sei(); // Re-enable global interrupts after configuration
}

// Common function to play a buzzer sound using different sets of frequency and duration arrays. 
void playBuzzer(int tune[], int duration[], int numNotes) {
    int i, j;  // Loop counters for notes and durations

    for (i = 0; i < numNotes; i++) {  // Loop through each note
        ICR3 = tune[i];           // Set the frequency of the note
        OCR3A = tune[i] / 2;      // Set the duty cycle (50%)
        TCNT3 = 0;                // Reset Timer 3

        for (j = 0; j < duration[i]; j++) {  // Loop through the duration of each note
            _delay_ms(1);         // Delay for the specified duration
        }
    }
    // Reset Timer 3 after playing the tune
    ICR3 = 0; OCR3A = 0; TCNT3 = 0;
}

// Function to play a cat-specific buzzer sound
void Catbuzzer() {
    int tune1[] = {200, 180, 80, 140, 120, 100, 250, 300, 400, 500, 600, 700, 800, 900, 1000}; // Frequencies
    int dur1[] = {50, 70, 90, 110, 130, 150, 100, 200, 300, 150, 200, 250, 300, 350, 400}; // Durations
    playBuzzer(tune1, dur1, sizeof(tune1) / sizeof(tune1[0]));  // Play the cat buzzer using the common function
}

// Function to play a dog-specific buzzer sound
void Dogbuzzer() {
    int tune1[] = {277, 311, 370, 554, 466, 415, 494, 370, 349, 330, 392, 523, 294, 440, 262, 587}; // Frequencies
    int dur1[] = {200, 100, 300, 400, 150, 150, 300, 400, 200, 100, 300, 400, 150, 150, 300, 400}; // Durations
    playBuzzer(tune1, dur1, sizeof(tune1) / sizeof(tune1[0]));  // Play the dog buzzer using the common function
}

// Function to dispense food for the cat
void dispenseCatFood() {
    setServoPosition(190);  // Open position
    _delay_ms(3000);        // Wait for food to dispense
    setServoPosition(10);   // Close position
    printf("Cat fed\n\r");
}

// Function to dispense food for the dog
void dispenseDogFood() {
    setServoPosition(190);  // Open position
    _delay_ms(3000);        // Wait for food to dispense
    setServoPosition(10);   // Close position
    printf("Dog fed\n\r");
}

// Check if it's time to feed the cat
bool shouldFeedCat() {
    // Returns true when the timeCounter is greater than 0 and a multiple of catInterval
    return (timeCounter > 0) && (timeCounter % catInterval == 0);
}

// Check if it's time to feed the dog
bool shouldFeedDog() {
    // Returns true when the timeCounter is greater than 0 and a multiple of dogInterval
    return (timeCounter > 0) && (timeCounter % dogInterval == 0);
}

// Function to feed the dog
void feedDog() {
    Dogbuzzer();  // Play the dog-specific buzzer sound
    int waitTime = 0;  // Initialize waitTime to track feeding attempts
    int condition = 5;  // Set the max number of feeding attempts to 5

    // Loop to check the sensor and attempt feeding within the given condition
    while (waitTime <= condition) {
        sensorValue = weightSensor();  // Read the weight sensor value
        printf("Weight sensor mean value: %d\n\r", sensorValue);  // Print the sensor value

        if (sensorValue >= 0 && sensorValue <= 50) {
            // If no pet is detected
            printf("None found\n\r");
        } else if (sensorValue >= 715 && sensorValue <= 785) {
            // If the sensor detects a dog, dispense food
            printf("Dog found\n\r");
            dispenseDogFood();  // Dispense dog food
            break;  // Exit the loop after successful feeding
        } else {
            // If a cat is detected, ignore and print "Cat found"
            printf("Cat found\n\r");
        }
        waitTime++;  // Increment the wait time counter
    }
}

// Function to feed the cat
void feedCat() {
    Catbuzzer();  // Play the cat-specific buzzer sound
    int waitTime = 0;  // Initialize waitTime to track feeding attempts
    int condition = 5;  // Set the max number of feeding attempts to 5

    // Loop to check the sensor and attempt feeding within the given condition
    while (waitTime <= condition) {
        sensorValue = weightSensor();  // Read the weight sensor value
        printf("Weight sensor mean value: %d\n\r", sensorValue);  // Print the sensor value

        if (sensorValue >= 0 && sensorValue <= 50) {
            // If no pet is detected
            printf("None found\n\r");
        } else if (sensorValue >= 240 && sensorValue <= 260) {
            // If the sensor detects a cat, dispense food
            printf("Cat found\n\r");
            dispenseCatFood();  // Dispense cat food
            break;  // Exit the loop after successful feeding
        } else {
            // If a dog is detected, ignore and print "Dog found"
            printf("Dog found\n\r");
        }
        waitTime++;  // Increment the wait time counter
    }
}

/* 
Interrupt Service Routine (ISR) for Timer 1 compare match.
This function gets triggered whenever Timer 1 reaches a specific compare value.
TCNT1 is reloaded with 0xC2F7 (49911 in decimal) to set the timer’s start point.
Each overflow increments the timeCounter to track elapsed time.
Tracks is tasks are performed correctly. 
Since this runs in the background, it won’t block the main loop.
*/
ISR(TIMER1_COMPC_vect) {
    TCNT1 = 0xC2F7; // Reload the Timer 1 counter to avoid overflow
    timeCounter++;  // Increment the time counter to track elapsed time
}

/* Main loop function continuously checks the weight sensor and feeds pets when required. */
void loop() {
    // Read the weight sensor value and store it in sensorValue
    sensorValue = weightSensor();  
    // Print the sensor value 
    printf("Mean value of the weight sensor: %d\n\r", sensorValue);

    // If it’s time to feed the dog, call feedAnimal 
    if (shouldFeedDog()) {
        feedAnimal("Dog", 715, 785, Dogbuzzer, dispenseDogFood);
    } 
    // If it’s time to feed the cat, call feedAnimal 
    else if (shouldFeedCat()) {
        feedAnimal("Cat", 240, 260, Catbuzzer, dispenseCatFood);
    }
}