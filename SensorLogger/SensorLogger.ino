/*!
    \file     SensorLogger.ino
    \brief    bluefruit_playground.ino, modified

    \author   Adafruit, Gemuele Aludino
    \date     22 May 2022

    This is an example for our nRF52 based Bluefruit LE modules

    Pick one up today in the adafruit shop!

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    MIT license, check LICENSE for more information
    All text above, and the splash screen below must be included in
    any redistribution

    This sketch demonstrate the BLE Adafruit Service that is used with
    "Adafruit Bluefruit Playground" app. Supported boards are
    - Circuit Playground Bluefruit : https://www.adafruit.com/product/4333
    - CLUE nRF52840 : https://www.adafruit.com/product/4500
    - Feather Sense : https://www.adafruit.com/product/4516
 */

#include "Adafruit_LittleFS.h"
#include "BLEAdafruitService.h"
#include "InternalFileSystem.h"
#include "PDM.h"
#include "SPI.h"
#include "SdFat.h"
#include "bluefruit.h"

#include <array>
#include <memory>

template <typename T>
constexpr void unused(const T &arg) { (void)(arg); }

/*!
    Circuit Playground Bluefruit
 */
#if defined(ARDUINO_NRF52840_CIRCUITPLAY)

///
/// Includes
///

#include <Adafruit_CircuitPlayground.h

///
/// Prototypes
///

uint16_t measure_temperature(uint8_t *buf, uint16_t bufsize);
uint16_t measure_light(uint8_t *buf, uint16_t bufsize);
uint16_t measure_button(uint8_t *buf, uint16_t bufsize);

///
/// Globals
///

constexpr auto DEVICE_NAME = "CPlay"
constexpr auto NEOPIXEL_COUNT = 10

///
/// Implementation
///

uint16_t measure_temperature(uint8_t *buf, uint16_t bufsize) {
    auto temp = CircuitPlayground.temperature();
    std::copy(&temp, &temp + 1, buf);
    return 4;
}

uint16_t measure_light(uint8_t *buf, uint16_t bufsize) {
    auto lux = float{};
    lux = CircuitPlayground.lightSensor();
    std::copy(&lux, &lux + 1, buf);
    return 4;
}

uint16_t measure_button(uint8_t *buf, uint16_t bufsize) {
    auto button = uint32_t{};

    button |= (CircuitPlayground.slideSwitch() ? 0x01 : 0x00);
    button |= (CircuitPlayground.leftButton() ? 0x02 : 0x00);
    button |= (CircuitPlayground.rightButton() ? 0x04 : 0x00);

    std::copy(&button, &button + 1, buf);
    return 4;
}

/*!
    CLUE & Feather Sense
 */
#elif defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)

///
/// Includes
/// 

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>

#include <Adafruit_AHRS.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor_Calibration.h>

///
/// Prototypes
///

void light_enable_callback(uint16_t conn_hdl, bool enabled);
uint16_t measure_light(uint8_t *buf, uint16_t bufsize);

void color_enable_callback(uint16_t conn_hdl, bool enabled);
uint16_t measure_color(uint8_t *buf, uint16_t bufsize);

void gesture_enable_callback(uint16_t conn_hdl, bool enabled);
uint16_t measure_gesture(uint8_t *buf, uint16_t bufsize);

void proximity_enable_callback(uint16_t conn_hdl, bool enabled);
uint16_t measure_proximity(uint8_t *buf, uint16_t bufsize);

///
/// Globals
///

#if defined(ARDUINO_NRF52840_CLUE)
constexpr auto DEVICE_NAME = "CLUE";
#else
constexpr auto DEVICE_NAME = "Sense";
#endif

constexpr auto NEOPIXEL_COUNT = 1;

auto ble_baro = BLEAdafruitBaro{};
auto ble_color = BLEAdafruitColor{};
auto ble_gesture = BLEAdafruitGesture{};
auto ble_humid = BLEAdafruitHumid{};
auto ble_proximity = BLEAdafruitProximity{};
auto ble_quarternion = BLEAdafruitQuaternion{};

auto barometer_service = BLEService(BLEAdafruitBaro::UUID128_SERVICE);
auto color_service = BLEService(BLEAdafruitColor::UUID128_SERVICE);
auto gesture_service = BLEService(BLEAdafruitGesture::UUID128_SERVICE);
auto humidity_service = BLEService(BLEAdafruitHumid::UUID128_SERVICE);
auto proximity_service = BLEService(BLEAdafruitProximity::UUID128_SERVICE);
auto quaternion_service = BLEService(BLEAdafruitQuaternion::UUID128_SERVICE);

auto lsm6ds33 = Adafruit_LSM6DS33{}; // Gyro and Accel
auto lis3mdl = Adafruit_LIS3MDL{};   // Magnetometer
auto apds9960 = Adafruit_APDS9960{}; // Proximity, Light, Gesture, Color
auto bmp280 = Adafruit_BMP280{};     // Temperature, Barometric
auto sht30 = Adafruit_SHT31{};       // Humid

// pick your filter! slower == better quality output
// auto filter = Adafruit_NXPSensorFusion{}; // slowest
// auto filter = Adafruit_Madgwick{};  // faster than NXP
auto filter = Adafruit_Mahony{}; // fastest/smallest

// Sensor calibration
constexpr auto FILE_SENSOR_CALIB = "sensor_calib.json";
auto cal = Adafruit_Sensor_Calibration_SDFat{};

auto flash_transport = Adafruit_FlashTransport_QSPI{};
auto flash = Adafruit_SPIFlash(&flash_transport);
auto fatfs = FatFileSystem{};

///
/// Implementation
///

void light_enable_callback(uint16_t conn_hdl, bool enabled) {
    unused(conn_hdl);
}

uint16_t measure_light(uint8_t *buf, uint16_t bufsize) {
    struct {
      uint16_t r{};
      uint16_t g{};
      uint16_t b{};
      uint16_t c{};
    } color;

    apds9960.getColorData(&color.r, &color.g, &color.b, &color.c);

    auto lux = static_cast<float>(color.c);
    std::copy(&lux, &lux + 1, buf);

    return 4;
}

void color_enable_callback(uint16_t conn_hdl, bool enabled) {
    unused(conn_hdl);
    apds9960.enableColor(enabled);

#ifdef ARDUINO_NRF52840_CLUE
    digitalWrite(PIN_LED2, enabled);
#endif
}

uint16_t measure_color(uint8_t *buf, uint16_t bufsize) {
    auto rgb = std::array<uint16_t, 3>{};
    auto c = uint16_t{};
    unused(c);

    apds9960.getColorData(rgb.data() + 0, rgb.data() + 1, rgb.data() + 2, &c);

    std::copy(rgb.begin(), rgb.end(), buf);
    return sizeof(rgb);
}

void gesture_enable_callback(uint16_t conn_hdl, bool enabled) {
    unused(conn_hdl);
    apds9960.enableProximity(enabled);
    apds9960.enableGesture(enabled);
}

uint16_t measure_gesture(uint8_t *buf, uint16_t bufsize) {
    auto gesture = apds9960.readGesture();

    if (gesture == 0) {
        return 0; // skip no gesture value
    }

    // APDS9960 sensor position is rotated 90 degree left on CLUE
    // We will need to correct that by rotating right for user convenience
    auto clue_rotation = std::array<uint8_t, 5>{0, APDS9960_LEFT, APDS9960_RIGHT,
                                     APDS9960_DOWN, APDS9960_UP};
    buf[0] = clue_rotation[gesture];

    return 1;
}

void proximity_enable_callback(uint16_t conn_hdl, bool enabled) {
    unused(conn_hdl);
    apds9960.enableProximity(enabled);
}

uint16_t measure_proximity(uint8_t *buf, uint16_t bufsize) {
    // APDS is only 8-bit, we better to map it to 16-bit value
    auto data8 = apds9960.readProximity();
    auto data16 = static_cast<uint16_t>(map(data8, 0, UINT8_MAX, 0, UINT16_MAX));

    std::copy(&data16, &data16 + 1, buf);
    return 2;
}

uint16_t measure_button(uint8_t *buf, uint16_t bufsize) {
    // Button is active LOW on most board except CPlay
    // No slide switch

    auto button = uint32_t{};
    button |= (digitalRead(PIN_BUTTON1) ? 0x00 : 0x02);
#if defined(PIN_BUTTON2)
    button |= (digitalRead(PIN_BUTTON2) ? 0x00 : 0x04);
#endif
    std::copy(&button, &button + 1, buf);
    return 4;
}

uint16_t measure_humid(uint8_t *buf, uint16_t bufsize) {
    auto humid = sht30.readHumidity();
    std::copy(&humid, &humid + 1, buf);
    return 4;
}

#else
#error "Board is not supported"

#endif // end of board

/*!
    Common for all boards
 */

///
/// Prototypes
///

uint16_t measure_button(uint8_t *buf, uint16_t bufsize);
uint16_t measure_humid(uint8_t *buf, uint16_t bufsize);
uint16_t measure_sound(uint8_t *buf, uint16_t bufsize);

void on_pdm_data();
void pdm_plotter(uint16_t count);

void start_advertising(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);

///
/// Globals
///

// BLE Service
auto ble_dfu = BLEDfu{};   // OTA DFU service
auto ble_dis = BLEDis{};   // device information
auto ble_uart = BLEUart{}; // uart over ble
auto ble_bas = BLEBas{};   // battery

// Adafruit Service: ADAFxxxx-C332-42A8-93BD-25E905756CB8
inline std::string adafruit_uuid(const std::string &unique_portion) {
    return std::string{"ADAF-" + unique_portion + "C332-42A8-93BD-25E905756CB8"};
}

auto ble_temperature = BLEAdafruitTemperature{};
auto ble_accelerometer = BLEAdafruitAccel{};
auto ble_light = BLEAdafruitLightSensor{};
auto ble_button = BLEAdafruitButton{};
auto ble_tone = BLEAdafruitTone{};
auto ble_pixel = BLEAdafruitAddressablePixel{};
auto ble_sound = BLEAdafruitSound{};

auto strip =
    Adafruit_NeoPixel{NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800};
auto pdm_sample = std::array<int16_t, 256>{};  // sound samples
auto pdm_byte_count = uint16_t{};

uint16_t measure_sound(uint8_t *buf, uint16_t bufsize) {
    const auto len = min(bufsize, pdm_byte_count);

    if (len) {
        std::copy(pdm_sample.begin(), pdm_sample.end(), buf);
    }

    pdm_byte_count = 0; // clear count

    // if (len) { ada_callback(NULL, 0, pdm_plotter, len); }

    return len;
}

void setup() {
    Adafruit_Sensor *accel_sensor = nullptr;

    Serial.begin(115200);
    // while(!Serial) { delay(10); }// wait for native USB

#if defined ARDUINO_NRF52840_CIRCUITPLAY
    CircuitPlayground.begin();
    accel_sensor = &CircuitPlayground.lis;
#else
    // Button
    pinMode(PIN_BUTTON1, INPUT_PULLUP);
#if defined(PIN_BUTTON2)
    pinMode(PIN_BUTTON2, INPUT_PULLUP);
#endif

#ifdef ARDUINO_NRF52840_CLUE
    // White LEDs for color sensing
    pinMode(PIN_LED2, OUTPUT);
    digitalWrite(PIN_LED2, LOW);
#endif

    apds9960.begin();
    bmp280.begin();
    sht30.begin(0x44);
    lsm6ds33.begin_I2C();
    lis3mdl.begin_I2C();

    // set lowest range
    lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    // set slightly above refresh rate
    lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    // Increase I2C speed to 400 Khz
    Wire.setClock(400000);

    accel_sensor = lsm6ds33.getAccelerometerSensor();

    // Init flash, filesystem and calibration & load calib json
    flash.begin();
    fatfs.begin(&flash);
    cal.begin(FILE_SENSOR_CALIB, &fatfs);
    cal.loadCalibration();
#endif

    // 1 channel (mono mode) with 16 kHz sample rate
    PDM.onReceive(on_pdm_data);
    PDM.begin(1, 16000);

    Serial.println("Bluefruit Playground Example");
    Serial.println("---------------------------\n");

    // Setup the BLE LED to be enabled on CONNECT
    // Note: This is actually the default behaviour, but provided
    // here in case you want to control this LED manually via PIN 19
    Bluefruit.autoConnLed(false);

    // Config the peripheral connection with maximum bandwidth
    // more SRAM required by SoftDevice
    // Note: All config***() function must be called before begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

    Bluefruit.begin();
    Bluefruit.setTxPower(8); // Check bluefruit.h for supported values
    Bluefruit.setName(DEVICE_NAME);
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    // To be consistent OTA DFU should be added first if it exists
    ble_dfu.begin();

    // Configure and Start Device Information Service
    ble_dis.setManufacturer("Adafruit Industries");
    ble_dis.begin();

    // Configure and Start BLE Uart Service
    ble_uart.begin();

    // Start BLE Battery Service
    ble_bas.begin();
    ble_bas.write(100);

    //------------- Adafruit Service -------------//
    ble_accelerometer.begin(
        accel_sensor,
        100); // TODO dropped in favor to Quaternion service for CLUE & Sense

    ble_button.begin(measure_button, 100);
    ble_button.setPeriod(0); // only notify if there is changes with buttons

    strip.begin();
    ble_pixel.begin(&strip);

    ble_sound.begin(1, measure_sound, 100);

    // CPB doesn't support these on-board sensor
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
    bleLight.begin(measure_light, 100);
    bleTemp.begin(measure_temperature, 100);

#else
    ble_baro.begin(bmp280.getPressureSensor(), 100);

    ble_color.begin(measure_color, 100);
    ble_color.setNotifyCallback(color_enable_callback);

    ble_gesture.begin(measure_gesture, 10); // sampling is 10 ms
    ble_gesture.setPeriod(0);               // notify on changes only
    ble_gesture.setNotifyCallback(gesture_enable_callback);

    ble_humid.begin(measure_humid, 100);

    ble_light.begin(measure_light, 100);
    ble_light.setNotifyCallback(light_enable_callback);

    ble_proximity.begin(measure_proximity, 100);
    ble_proximity.setNotifyCallback(proximity_enable_callback);

    // Quaternion with sensor calibration
    ble_quarternion.begin(&filter, accel_sensor, lsm6ds33.getGyroSensor(), &lis3mdl);
    ble_quarternion.setCalibration(&cal);

    ble_temperature.begin(bmp280.getTemperatureSensor(), 100);
#endif

#if defined(PIN_BUZZER)
    ble_tone.begin(PIN_BUZZER);
#endif

    // Set up and start advertising
    start_advertising();

    Serial.println(
        "Please use Adafruit's Bluefruit LE app to connect in UART mode");
    Serial.println("Once connected, enter character(s) that you wish to send");
}

void loop() {}

void on_pdm_data() {
    // query the number of bytes available
    pdm_byte_count = PDM.available();

    // read into the sample buffer
    PDM.read(pdm_sample.data(), pdm_byte_count);
}

void pdm_plotter(uint16_t count) {
    for (int i = 0; i < count / 2; i++) { 
        Serial.println(pdm_sample[i]); 
    }
}

void start_advertising(void) {
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

    // Advertising with only board ID
    struct ATTR_PACKED {
        uint16_t mfr_id{};

        uint8_t field_len{};
        uint16_t field_key{};
        uint16_t field_value{};
    } mfr_adv;

    mfr_adv.mfr_id = UUID16_COMPANY_ID_ADAFRUIT;
    mfr_adv.field_len = 4;
    mfr_adv.field_key = 1; // board id
    mfr_adv.field_value = USB_PID;

    Bluefruit.Advertising.addManufacturerData(&mfr_adv, sizeof(mfr_adv));

    // Add name to advertising, since there is enough room
    Bluefruit.Advertising.addName();

    // Add services for advertising
    Bluefruit.Advertising.addService(barometer_service);
    Bluefruit.Advertising.addService(color_service);
    Bluefruit.Advertising.addService(gesture_service);
    Bluefruit.Advertising.addService(humidity_service);
    Bluefruit.Advertising.addService(proximity_service);
    Bluefruit.Advertising.addService(quaternion_service);

    /*! 
        Start Advertising
        - Enable auto advertising if disconnected
        - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
        - Timeout for fast mode is 30 seconds
        - Start(timeout) with timeout = 0 will advertise forever (until connected)
     
        For recommended advertising interval
        https://developer.apple.com/library/content/qa/qa1931/_index.html
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
    Bluefruit.Advertising.start(0); 
    // 0 = Don't stop advertising after n seconds
}

/*!
    \brief  Callback invoked when central connects
    \param  conn_handle connection where this event happens
 */
void connect_callback(uint16_t conn_handle) {
    // Get the reference to current connection
    auto connection = Bluefruit.Connection(conn_handle);

    auto central_name = std::array<char, 32>{};
    connection->getPeerName(central_name.data(), sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name.data());
}

/*!
    \brief  Callback invoked when a connection is dropped

    \param  conn_handle connection where this event happens
    \param  reason  is a BLE_HCI_STATUS_CODE whic hcan be found in `ble_hci.h`
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    unused(conn_handle);
    unused(reason);

#if defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
    apds9960.enableGesture(false);
    apds9960.enableProximity(false);
    apds9960.enableColor(false);
#endif

#ifdef ARDUINO_NRF52840_CLUE
    digitalWrite(PIN_LED2, LOW);
#endif

    Serial.println();
    Serial.print("Disconnected, reason = 0x");
    Serial.println(reason, HEX);
}
