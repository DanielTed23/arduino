#include <DHT11.h>
#include <EEPROM.h>

//int ANALOG_PIN A0       // Pin for the analog sensor (potentiometer)
//int DHT_PIN 2           // Pin for the DHT11 sensor
int BLUE_LED_PIN = 5;      // Pin for blue LED (temperature too high)
int GREEN_LED_PIN = 4;     // Pin for green LED (temperature correct)
int RED_LED_PIN = 6;       // Pin for red LED (temperature too low)
int BUTTON_PIN = 7;        // Pin for the toggle button
int YELLOW_LED_PIN = 8;    // Pin for yellow LED (thermostat off)

DHT11 dht11(2);

bool thermostatOn = true;           // Variable to track thermostat state
unsigned long lastDebounceTime = 0;  // Time for debounce check
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds
const int thermostatOnadd = 0;
bool buttonState = LOW;              // Current button state
bool lastButtonReading = LOW;        // Previous button state for comparison

void setup() {
    Serial.begin(9600);

    // Set LED pins as output
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    thermostatOn = EEPROM.read(thermostatOnadd);
    // Set button pin as input
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    Serial.println(thermostatOn);
}

void loop() {
    // Read the button state
    bool reading = digitalRead(BUTTON_PIN);
    EEPROM.update(thermostatOnadd, thermostatOn);
    // Check if button state has changed
    if (reading != lastButtonReading) {
        lastDebounceTime = millis();  // Reset debounce timer if state has changed
    }

    // Check if debounce time has passed and state is stable
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // If the button state has changed and is stable, toggle thermostat state
        if (reading != buttonState) {
            buttonState = reading;

            // Only toggle thermostat state if button is LOW (pressed)
            if (buttonState == LOW) {
                thermostatOn = !thermostatOn;  // Toggle the thermostat state
                Serial.print("Thermostat state toggled to: ");
                Serial.println(thermostatOn ? "ON" : "OFF");
            }
        }
    }

    // Update the last button reading for the next loop
    lastButtonReading = reading;

    // Check if thermostat is on or off
    if (thermostatOn) {
        // Turn off yellow LED if thermostat is on
        digitalWrite(YELLOW_LED_PIN, LOW);

        // Read the potentiometer value to set the target temperature
        int potValue = analogRead(A0);
        int targetTemperature = map(potValue, 0, 1023, 15, 30); // Adjust range as needed (15-30 °C here)

        // Print the target temperature for debugging
        Serial.print("Target Temperature: ");
        Serial.println(targetTemperature);

        // Read temperature and humidity from the DHT11 sensor
        int temperature, humidity;
        int result = dht11.readTemperatureHumidity(temperature, humidity);

        // Check if the reading is successful
        if (result == 0) {
            // Print temperature and humidity values
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.print(" °C, Humidity: ");
            Serial.print(humidity);
            Serial.println(" %");

            // Set a tolerance range of ±1°C around the target temperature
            int tolerance = 1;

            // Control LEDs based on the temperature comparison
            if (temperature < targetTemperature - tolerance) {
                // Temperature is lower than target - turn on the red LED
                digitalWrite(RED_LED_PIN, HIGH);
                digitalWrite(GREEN_LED_PIN, LOW);
                digitalWrite(BLUE_LED_PIN, LOW);
            } else if (temperature > targetTemperature + tolerance) {
                // Temperature is higher than target - turn on the blue LED
                digitalWrite(RED_LED_PIN, LOW);
                digitalWrite(GREEN_LED_PIN, LOW);
                digitalWrite(BLUE_LED_PIN, HIGH);
            } else {
                // Temperature is within the target range - turn on the green LED
                digitalWrite(RED_LED_PIN, LOW);
                digitalWrite(GREEN_LED_PIN, HIGH);
                digitalWrite(BLUE_LED_PIN, LOW);
            }
        } else {
            // Print an error message if the reading fails
            Serial.print("DHT11 Error: ");
            Serial.println(DHT11::getErrorString(result));
        }
    } 
    else {
        // If thermostat is off, turn on yellow LED and turn off others
        digitalWrite(YELLOW_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, LOW);
    }

    // Add a delay before the next reading
    delay(200);
}
