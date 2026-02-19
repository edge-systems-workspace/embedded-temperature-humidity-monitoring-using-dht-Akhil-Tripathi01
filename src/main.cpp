#include <Arduino.h>
/**
 * @file main.cpp
 * @brief Embedded Temperature and Humidity Monitoring System using DHT11 Sensor
 * @author Akhil Tripathi
 * @date 2026-02-19
 * @version 1.0
 *
 * @mainpage DHT11 Temperature and Humidity Monitor
 *
 * @section Description
 * This embedded system application monitors environmental conditions using a DHT11
 * digital temperature and humidity sensor. The system continuously reads sensor data
 * at regular intervals and transmits the measurements via Serial communication to a
 * host device for real-time monitoring and logging.
 *
 * @section Hardware Requirements
 * - Arduino microcontroller (Arduino Uno, Arduino Nano, or compatible board)
 * - DHT11 Temperature and Humidity Sensor
 * - 4.7kΩ resistor (pull-up resistor for data line)
 * - USB cable for serial communication
 * - Power supply (5V)
 *
 * @section Specifications
 * - **DHT11 Temperature Range**: 0°C to 50°C (±2°C accuracy)
 * - **DHT11 Humidity Range**: 20% to 90% RH (±5% accuracy)
 * - **Data Pin**: Digital Pin 2
 * - **Sampling Rate**: 2 seconds between readings
 * - **Serial Baud Rate**: 9600 bps
 * - **Output Format**: "Temperature: XX.XX°C, Humidity: XX.XX%"
 *
 * @section Dependencies
 * - DHT Sensor Library (Adafruit DHT Unified Sensor Library)
 * - Arduino Core Library
 *
 * @section Usage
 * 1. Connect DHT11 sensor to Arduino Pin 2 (data), Pin 5V (power), and GND (ground)
 * 2. Upload the sketch to the Arduino board
 * 3. Open Serial Monitor at 9600 baud rate
 * 4. Temperature and humidity values will be displayed every 2 seconds
 *
 * @note Ensure proper sensor initialization before reading values to avoid invalid data
 * @warning Do not exceed operating specifications of the DHT11 sensor
 * @see https://www.adafruit.com/product/386 (DHT11 Sensor Specification)
 */

#include <DHT.h>

/** @brief Digital pin number for DHT11 data line connection */
#define DHTPIN 2

/** @brief DHT sensor type identifier for DHT11 model */
#define DHTTYPE DHT11

/** @brief Global DHT sensor object instance
 *  Initialized with DHTPIN and DHTTYPE to communicate with the sensor
 */
DHT dht(DHTPIN, DHTTYPE);

/**
 * @brief Initialize the Arduino microcontroller and DHT sensor
 *
 * This function is called once when the microcontroller starts or resets.
 * It performs the following initialization tasks:
 * - Initializes Serial communication at 9600 baud rate for debug output
 * - Initializes the DHT11 sensor for temperature and humidity measurements
 * - Prints a startup message to confirm successful initialization
 *
 * @note This function must be called before any sensor operations
 * @note Serial Monitor should be opened at 9600 baud rate to view output
 * @warning Ensure DHT sensor is properly connected before calling this function
 *
 * @return void
 *
 * @see dht.begin()
 * @see Serial.begin()
 */
void setup() {

    Serial.begin(9600);

    dht.begin();

    Serial.println("DHT11 Sensor Initialized!");
}

/**
 * @brief Main program loop - continuously reads and outputs sensor data
 *
 * This function runs repeatedly after setup() completes. It performs the
 * following operations in each cycle:
 *
 * 1. **Sensor Reading**: Acquires current humidity and temperature values
 *    from the DHT11 sensor
 *
 * 2. **Data Validation**: Checks if sensor readings are valid using isnan()
 *    to detect failed read operations. Common failure causes include:
 *    - Loose or disconnected sensor wiring
 *    - Insufficient pull-up resistor on data line
 *    - Electromagnetic interference near sensor
 *
 * 3. **Data Output**: Transmits validated temperature and humidity values
 *    in formatted text to Serial Monitor at 9600 baud rate
 *
 * 4. **Timing Control**: Implements a 2-second delay between consecutive
 *    sensor readings to allow adequate sensor recovery time
 *
 * @note The DHT11 sensor requires minimum 1 second between reads for accurate data
 * @note Serial output format: "Temperature: XX.XX°C, Humidity: XX.XX%"
 * @note Temperature values are in Celsius (°C)
 * @note Humidity values are in relative humidity percentage (%)
 *
 * @par Error Handling
 * If either humidity or temperature reading returns NaN (Not a Number),
 * the function prints an error message and exits early without processing
 * invalid data.
 *
 * @return void
 *
 * @see dht.readHumidity()
 * @see dht.readTemperature()
 * @see isnan()
 * @see Serial.print()
 * @see delay()
 *
 * @par Example Output
 * ```
 * Temperature: 24.50°C, Humidity: 65.00%
 * Temperature: 24.51°C, Humidity: 64.98%
 * Temperature: 24.52°C, Humidity: 64.95%
 * ```
 */
void loop() {

    float humidity = dht.readHumidity();

    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");

    delay(2000);
}
