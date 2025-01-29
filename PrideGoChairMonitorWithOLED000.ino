#define DEBUG_SER_MON   // Send Data to Serial Device for Debug
//#define LCD_DISP        // Use LCD Display
#define OLED_DISP       // Use OLED Display

// My current batch of 1.3 inch OLED displays have a problem displaying
// Pixels in the 1st 2 columns.  (This was my easy fix.)
const int OLED_OFFSET = 2;

/******************************************************************************
  MOTERIZED WHEELECHAIR/BICYCLE SPEEDOMETER/ODOMETER
*******************************************************************************
  Created:  October 2024
  By:       ChatGPT and Stephen A. Brown
  Update:   January 28, 2025
*******************************************************************************
  Updates & Error Resolutions

  01/26/2025 - Added debounce to hallISR() to avoide erronious Speed & 
  Max Speed indications.
  
*******************************************************************************
This code is for an **ESP32-based** motorized wheelchair/bicycle 
speedometer/odometer project. It tracks speed and distance using a Hall effect 
sensor and displays data on a 20x4 I2C LCD and/or 1.3 inch monicrome OLED. The 
system can connect to Wi-Fi to synchronize the RTC (Real-Time Clock) using NTP 
(Network Time Protocol). Additionally, a button allows resetting and updating 
the RTC time via Wi-Fi. 

The code is compiled conditionally to allow data to be displayed
on any combination of 3 different devices by defining the appropriate variable:

  DEBUG_SER_MON:    Serial Monitor
  LCD_DISP:         20 x 4 LCD Display. (Large, Easy to See)
  OLED_DISP:        1.3 inch monochrome OLED Display. (Low Power)

Here's an overview of the key components and functionalities:

### Key Hardware Components:

1. **ESP32**: The main microcontroller handling Wi-Fi connectivity, data 
    processing, and I/O operations. For conveniance, We use an
    ACEBOTT ESP32 Max V1.0 Development Board.

2. **Hall Effect Sensor (Pin 13)**: Detects wheel rotations by measuring 
    magnetic field changes, used for calculating speed and distance.

3. **Push Button (Pin 12)**: Resets the RTC time via Wi-Fi. Active low logic 
    with `INPUT_PULLUP`.
      Short Push --> Software Reset
      Medium Push (<3 Sec.) --> Set RTC from WiFi
      Long Push (>3 Sec.) --> Reset SSID, PWD, TZ, Wheel Diameter
                              causing WiFiManager to get new settings

4. **20x4 I2C LCD (Address 0x27)**: Displays real-time data including speed, 
    distance, and time. Connected via I2C (GPIO 21 for SDA, GPIO 22 for SCL).

5. ** 1.3 inch OLED (Address 0x__) **: Displays real-time data including 
    speed, distance, and time. Connected via I2C (GPIO 21 for SDA, GPIO 22 
    for SCL).

6. **DS3231 RTC**: Keeps track of the current date and time, synchronized via  
    Wi-Fi using the NTP server (`pool.ntp.org`). The Built-In RTC is NOT
    Battery Back-Up.

7. **Wi-Fi Module**: Enables connection to a Wi-Fi network for NTP 
    synchronization.

### Libraries Used:

- `Wire.h`: I2C communication for the LCD, OLED and RTC.
- `LiquidCrystal_I2C.h`: Controls the I2C-based 20x4 LCD.
- 'U8g2lib.h': Controls the I2C-based U8g2 OLED.
- `WiFi.h`: Manages Wi-Fi connections.
- `RTClib.h`: Communicates with the DS3231 RTC module.
- `WiFiUdp.h`: Handles UDP connections for NTP.
- `NTPClient.h`: Fetches the current time from an NTP server.
- `WiFiManager.h`: Simplifies Wi-Fi credential management and auto-connects 
    to the network.
- `Preferences.h`: Used for saving non-volatile data:
    SSID
    Password
    Time Zone Offset
    Wheele Diamiter

### Main Functionalities:

1. **Hall Effect Sensor Interrupt (ISR)**: Monitors wheel revolutions using a 
    hardware interrupt on the falling edge of the Hall sensor's output. This 
    ensures no revolutions are missed during execution of other code.

2. **Wi-Fi Connection and NTP Synchronization**: On reset, the code attempts 
    to connect to a Wi-Fi network and synchronize the RTC via NTP. A time zone 
    offset is stored in non-volatile storage (NVS) and can be modified via a 
    Wi-Fi manager portal.

2A. Holding the Time Reset Button for 3 Seconds wipes NVS memory including:
      -> SSID (Network ID)
      -> Network Password
      -> Time Zone Offset
      -> Wheel Diameter
    forcing the unit to it starts an access point with the specified name:
      ->  ("AutoConnectAP"),
    it will auto generate SSID, if password is blank it will be anonymous AP 
    (wm.autoConnect()). Then goes into a blocking loop awaiting configuration
    and will return success result.

3. **LCD Display**: The LCD continuously displays time, speed, and distance, 
    with updates occurring once per second.

3. **OLED Display**: The OLED continuously displays time, speed, and distance, 
    with updates occurring once per second.

4. **Time Zone Handling**: The user can specify a time zone offset through 
    the Wi-Fi manager, which is saved to NVS for future use.

5. **RTC Management**: The system uses the DS3231 RTC for timekeeping and 
    synchronizes it with NTP to ensure accuracy.

### Code Breakdown:

1. **Interrupt Service Routine (ISR)**: 
   - `hallISR()`: Increments the revolution counter (`revCount`) each time the 
      Hall sensor detects a magnetic field change (wheel rotation).
   
2. **Time Zone Offset Management**:
   - `saveTimeZoneOffset()`: Stores the time zone offset in non-volatile memory.
   - `loadTimeZoneOffset()`: Retrieves the stored time zone offset from memory.
   
3. **NTP Time Synchronization**:
   - The RTC time is updated using NTP when the push button is pressed. The 
      time zone offset is applied to ensure local time is displayed.
   
4. **Main `setup()` Function**:
   - Initializes the serial monitor, LCD, OLED, RTC, Hall effect sensor, 
      and clock reset button.

   - Handles the Wi-Fi connection process for RTC synchronization if the reset 
      button is pressed.
   
5. **Main `loop()` Function**:
   - Updates the speed, distance, and runtime once per second based on the RTC 
      time.

   - Displays the current time, date, and runtime on the LCD and OLED.

This code is structured to be modular, allowing easy modification or extension 
of functionalities, such as additional sensor inputs or display features. 
The `DEBUG_SER_MON` directive can be used to enable or disable serial debugging
output to the Serial Monitor.

******************************************************************************/

#include <Wire.h>         // I2C Communications

#ifdef LCD_DISP
#include <LiquidCrystal_I2C.h>
#endif

#ifdef OLED_DISP
#include <U8g2lib.h>      // Include the U8g2 library for OLED
#endif

#include <WiFi.h>
#include <RTClib.h>       // Include the DS3231 RTC library
#include <WiFiUdp.h>      // Include for NTPa
#include <NTPClient.h>    // Include for NTP Client
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <time.h>         // Time library
#include <Preferences.h>  // Preferences library for NVS storage

//*****************************************************************************
//  Function Prototypes
//*****************************************************************************
void setup( void );
void loop( void );
void IRAM_ATTR hallISR( void ); // IRAM_ATTR --> function should be stored in 
                                // IRAM (Internal RAM) rather than Flash

void saveTimeZoneOffset( int offset );
int loadTimeZoneOffset( void );

void saveMaxSpeed( float maxSpeed );
float loadMaxSpeed( void );

void saveRunTimeSecs( unsigned long runTime );
unsigned long loadRunTimeSecs( void );

void saveTotalDistance( float distance );
float loadTotalDistance( void );

void saveWheelDiameter( float diameter );
float loadWheelDiameter( void );

void updateSpeedDistRuntime( void ); 
String getDayOfWeek( uint8_t wday );

//*****************************************************************************
//  Constants
//*****************************************************************************
const int DEFAULT_TIME_ZONE_OFFSET = 0;
const char* NTP_SERVER = "pool.ntp.org";  // NTP server

//const float DEFAULT_WHEEL_DIAMETER = 9.0; // Default wheel diameter in inches
                                          // Pride GoChair Drive Wheele
const float DEFAULT_WHEEL_DIAMETER = 8.0; // Default wheel diameter in inches
                                          // Pride GoGo 3 Wheel Traveler Elete 
                                          // Drive & Front Wheeles

const int OFF = 0;
const int ON  = 1;

const int IDLE_TIME_LOW_POWER = 180;      // Seconds Idle for Low Power Mode

Preferences preferences;  // Create a Preferences object
                          // For NVS (Non Volital Storage)

//
// Create a custom parameters for WiFi Manager
//
WiFiManagerParameter *custom_tz_offset;   // Custome Time Zone Offset
WiFiManagerParameter *custom_wheel_diameter;  // Custom Wheel Diameter

//
// LCD Setup (Main Display)
//
#ifdef LCD_DISP
LiquidCrystal_I2C lcd(0x27, 20, 4);  // I2C address, columns, rows
#endif

//
// OLED Setup (Main Display 2)
//
#ifdef OLED_DISP
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

//*****************************************************************************
// Clock Reset Button
//*****************************************************************************
const int CLOCK_RST_BTN = 12;

//*****************************************************************************
// Hall Effect Sensor
//*****************************************************************************
const int HALL_PIN = 13;
/*volitile*/ unsigned long lastDebounceTime = 0; // Time of Last valid pulse
/*volatile*/ unsigned long revCount = 0;
/*volatile*/ unsigned long idleCount = 0;

const unsigned long debounceDelay = 50; // Debounce delay in milliseconds

//*****************************************************************************
// Analog pin definitions
//*****************************************************************************
const int voltagePin =   34;   // Pin to read battery voltage

// Constants for voltage divider
const float R1 =     10000.0; 
const float R2 =      1000.0; // Variable Resistor, wiper to voltagePin
const float R3 =       330.0;

const int adcResolution = 4096;   // ESP32 ADC max value (12-bit resolution)
const float adcVoltage  =    3.3; // ESP32 reference voltage (V)
// Variables
float batteryVoltage =    0.0; // Variable to store measured battery voltage

//*****************************************************************************
// NTP Setup
//*****************************************************************************
WiFiUDP ntpUDP;

//*****************************************************************************
// Create an RTC object
//*****************************************************************************
RTC_DS3231 rtc;

//*****************************************************************************
// Global Variables
//*****************************************************************************
float currentSpeed        = 0.0;
float maxSpeed            = 0.0;
float totalDistance       = 0.0;
unsigned long runTimeSecs = 0;

int tzOffset              = 0;    // time zone offset
float wheelDiameter       = 0.0;  // wheel diameter

int ResetNVS = false;   // Reset Non Volital Storage parameters
                        // SSID (Name of Wi-Fi network)
                        // Password
                        // Time Zone Offset (Hours)
                        // Wheel Diameter (Inches)

//*****************************************************************************
//  setup() Function
//*****************************************************************************
void setup( void ) 
{
  //
  // Start Serial Monitor (For Debug)
  //
#ifdef DEBUG_SER_MON
  Serial.begin(115200);
#endif

  //
  //  Initialize Voltage Measurement Input Pin
  //
  pinMode(voltagePin, INPUT);

  //
  // Load global variables from NVS
  //
  maxSpeed      = loadMaxSpeed();
  runTimeSecs   = loadRunTimeSecs();
  totalDistance = loadTotalDistance();

  tzOffset      = loadTimeZoneOffset();
  wheelDiameter = loadWheelDiameter();

#ifdef LCD_DISP
  //
  // Initialize LCD (Main Display)
  //
  lcd.init();
  lcd.backlight();
#endif

#ifdef OLED_DISP
  //
  // Initialize OLED display
  //
  oled.begin();
//  oled.setDisplayRotation(U8G2_R2); // Rotate 180 degrees
  oled.clearBuffer();  // Clear the display buffer
  oled.setFont(u8g2_font_ncenB08_tr);  // Choose a suitable font
  oled.sendBuffer();  // Send the data to the display
#endif
//  delay(5000);

  //
  // Setup Hall Effect Sensor
  //
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(  digitalPinToInterrupt(HALL_PIN), 
                    hallISR,  // ISR handeler
                    FALLING); // Trigger Int. on Falling Edge of Pulse

  //
  // Setup Clock Reset Button 
  //
  pinMode(CLOCK_RST_BTN, INPUT_PULLUP);

  //
  // Initialize the DS3231 RTC
  //
  //  If it Fails to Initialize
  //    Display Error and loop doing nothing
  //
  if( !rtc.begin() )
  {
    //
    //  RTC Initialization FAILED!
    //
#ifdef DEBUG_SER_MON
    Serial.printf("\nRTC Not Found!!!");
#endif

#ifdef LCD_DISP
    lcd.clear();
    lcd.printf("RTC Not Found!!!");
#endif

#ifdef OLED_DISP
    oled.clearBuffer();
    oled.drawStr(OLED_OFFSET + 0, 10, "RTC Not Found!!!");
    oled.sendBuffer();
#endif

    while( 1 )
    {
      ;  // No RTC: No sense doing anything else.

    } // End of while()
  
  } // End of if()

  //
  //  RTC Found & Initialized
  //
  else
  {
    //
    //  RTC Initialization Suceeded!
    //

    //
    //  If Clock Reset Button is Pushed(Low)
    //    Reset RTC from Wi-Fi
    //
    if( !digitalRead(CLOCK_RST_BTN) )
    {
      //
      //  If CLOCK_RST_BTN is pushed for 3 seconds
      //    Flag to Reset NVS including:
      //      SSID (Network ID)
      //      Network Password
      //      Time Zone Offset
      //      Wheel Diameter
      //
      delay(3000);

      if( !digitalRead(CLOCK_RST_BTN) )
      {
        ResetNVS = true;

      } // End of if()

#ifdef DEBUG_SER_MON
      Serial.printf("\n\nConnecting Wi-Fi\n");
      Serial.printf("Update SSID & PWD: ");
      Serial.printf(" AutoConnectAP: ");
      Serial.printf(" 192.168.4.1 \n");
#endif

#ifdef LCD_DISP
      lcd.printf("Connecting Wi-Fi ");
      lcd.setCursor(0, 1);
      lcd.printf("Update SSID & PWD: ");
      lcd.setCursor(0, 2);
      lcd.printf(" AutoConnectAP ");
      lcd.setCursor(0, 3);
      lcd.printf(" 192.168.4.1 ");
#endif

#ifdef OLED_DISP
      oled.clearBuffer();
      oled.drawStr(OLED_OFFSET + 0, 10, "Connecting WiFi ");
      oled.drawStr(OLED_OFFSET + 0, 20, "Update SSID & PWD: ");
      oled.drawStr(OLED_OFFSET + 0, 30, " AutoConnectAP ");
      oled.drawStr(OLED_OFFSET + 0, 40, " 192.168.4.1 ");
      oled.sendBuffer();
#endif
      //
      //  Zero NVS Globals
      //
#ifdef DEBUG_SER_MON
      Serial.printf("Zeroing NVS\n");
#endif

      maxSpeed = 0.0;
      saveMaxSpeed( maxSpeed );
      runTimeSecs = 0;
      saveRunTimeSecs( runTimeSecs );
      totalDistance = 0.0;
      saveTotalDistance( totalDistance );

      //
      // Convert Time Zone Offset and Wheel Diameter
      // to Strings for WiFiManager Custom Defaults.
      //
      String tzOffsetStr = String(tzOffset);
      String wheelDiameterStr = String(wheelDiameter);

      //
      // WiFiManager, Local intialization. 
      // Once its business is done, 
      // there is no need to keep it around
      //
      WiFiManager wm;

      //
      // Add the custom timezone parameter to the WiFiManager portal
      //
      // Because we use new here for allocation, we need to delete the 
      // allocation when we are finished, to avoid a memory leak.
      //
      custom_tz_offset = new WiFiManagerParameter(  "tz_offset", 
                                                    "Time Zone Offset (in hours)", 
                                                    tzOffsetStr.c_str(), 
                                                    5);
       wm.addParameter(custom_tz_offset);

      //
      // Add the custom Wheel Diameter parameter to the WiFiManager portal
      //
      // Because we use new here for allocation, we need to delete the 
      // allocation when we are finished, to avoid a memory leak.
      //
      custom_wheel_diameter = new WiFiManagerParameter(  "wheel_diameter",
                                                  "Wheel Diameter (in inches)",
                                                  wheelDiameterStr.c_str(),
                                                  5);
       wm.addParameter(custom_wheel_diameter);

      //
      // Reset Settings - wipe stored credentials
      // these are stored by the ESP32 WiFiManager library.
      //
      if( ResetNVS == true )
      {
        //
        //  Reset NVS including:
        //    SSID (Network ID)
        //    Network Password
        //    Time Zone Offset
        //    Wheel Diameter
        //
        wm.resetSettings();
      
      } // End of if()

      //
      // Automatically connect using saved credentials:
      //
      // If connection fails, it starts an access point with the specified name
      //  ("AutoConnectAP"),
      //
      // If empty will auto generate SSID, if password is blank it will be 
      //  anonymous AP (wm.autoConnect())
      //
      // Then goes into a blocking loop awaiting configuration and will return
      //  success result

      bool AutoConnectResult = wm.autoConnect("AutoConnectAP"); // anonymous ap

      if( AutoConnectResult ) 
      {
        //
        //  if you get here you have connected to the WiFi    
        //
#ifdef DEBUG_SER_MON
        Serial.printf("Wi-Fi Connected!\n");
        Serial.printf("Setting RTC\n");
#endif

#ifdef LCD_DISP
        lcd.clear();
        lcd.printf("Wi-Fi Connected! ");
        lcd.setCursor(0, 1);
        lcd.printf("Setting RTC");
#endif

#ifdef OLED_DISP
        oled.clearBuffer();
        oled.drawStr(OLED_OFFSET + 0, 10, "Wi-Fi Connected!");
        oled.drawStr(OLED_OFFSET + 0, 20, "Setting RTC");
        oled.sendBuffer();
#endif

        // Retrieve the timezone offset entered by the user
        String tz_offset = custom_tz_offset->getValue();
        long gmtOffset_sec = 3600 * tz_offset.toInt();  // Convert to seconds

        // Convert the captured offset to an integer
        tzOffset = String(custom_tz_offset->getValue()).toInt();
        saveTimeZoneOffset(tzOffset);

#ifdef DEBUG_SER_MON
        Serial.printf("Entered Time Zone Offset: %i\n", tzOffset);
#endif

        // Retrieve the wheel diameter entered by the user
        String wheelDiameterStr = custom_wheel_diameter->getValue();
        wheelDiameter = wheelDiameterStr.toFloat();
        saveWheelDiameter(wheelDiameter);

#ifdef DEBUG_SER_MON
        Serial.printf("Entered Wheel Diameter: %.1f\n", wheelDiameter);
#endif

        //
        // Initialize the NTP client
        //
        NTPClient timeClient( ntpUDP, 
                            NTP_SERVER, 
                            gmtOffset_sec,  // UTC offset in seconds
                            60000);         // Update every 60 seconds

        timeClient.begin();
        timeClient.update();

        //
        // Get current time from NTP
        //
        timeClient.update();
        unsigned long epochTime = timeClient.getEpochTime();
        DateTime now = DateTime(epochTime);
        rtc.adjust(now); // Pass the DateTime object

#ifdef DEBUG_SER_MON
        Serial.printf("TZ    = %i ", tzOffset);
        Serial.printf("Wheel = %.2f ", wheelDiameter);
        Serial.printf("*** done! ***\n");
#endif

#ifdef LCD_DISP
        lcd.clear();
        lcd.printf("TZ    = %i ", tzOffset);
        lcd.setCursor(0, 1);
        lcd.printf("Wheel = %.2f", wheelDiameter);
        lcd.setCursor(0, 3);
        lcd.printf("*** done! ***");
#endif

#ifdef OLED_DISP
        oled.clearBuffer();
        oled.setCursor(OLED_OFFSET + 0, 10);
        oled.printf("TZ    = %i", tzOffset);  // Print the timezone offset
        oled.setCursor(OLED_OFFSET + 0, 20);
        oled.printf("Wheel = %.2f", wheelDiameter);  // Print the wheel diameter
        oled.setCursor(OLED_OFFSET + 0, 40);
        oled.printf("*** done! ***");  // Print the "done" message
        oled.sendBuffer();
#endif
        delay(3000);
    
      } // End of if()

      else
      {
#ifdef DEBUG_SER_MON
        Serial.printf("\nWi-Fi FAILED! /n");
#endif

#ifdef LCD_DISP
        lcd.printf("Wi-Fi FAILED! ");
#endif

#ifdef OLED_DISP
        oled.clearBuffer();               // Clear the OLED buffer
        oled.setCursor(OLED_OFFSET + 0, 10);
        oled.printf("Wi-Fi FAILED!");     // Print the message
        oled.sendBuffer();                // Send the buffer to the display
#endif

        delay(3000);
    
      } // End of else

      //
      //  Because we allocated some memory space with:
      //    new
      //  We need to release it with:
      //    delete
      //  to adhear to sound programming practices.
      //
      delete  custom_tz_offset;
      delete  custom_wheel_diameter;
 
    } // End of if()
    
#ifdef LCD_DISP
    lcd.clear();
#endif

#ifdef OLED_DISP
    oled.clearBuffer();  // Clear the internal buffer (clears the screen)
    oled.sendBuffer();   // Send the empty buffer to the display (refreshes it)
#endif

  } // End of if()
  
} // End of setup()

//*****************************************************************************
//  loop() Function
//
//  This uses the RTC Time for updating things.  It should be more accurate
//  than using millis().
//
//*****************************************************************************
void loop( void ) 
{
  static  DateTime previous = rtc.now();  // Initialize DateTime for Comparison

  //
  //  If Clock Reset Button is Pushed(Low)
  //    Reset RTC from Wi-Fi
  //
  //  *** NOTE ***
  //  This is as Brute Force and Unelegant as it gets. But all the logic needed
  //  is in the setup() function as that's where it originally belonged.
  //
  if( !digitalRead(CLOCK_RST_BTN) )
  {
    ESP.restart();
  
  } // End of if()
 
  //
  //  This is what makes everything else work.
  //
  //    If time has changed:  do everything
  //    Else:                 do nothing
  //
  DateTime now = rtc.now(); // Get the current time from the RTC

  if( now != previous )     // If time has changed, process...
  {
    previous = now;         // Update Previous to Current time
    updateNVS();            // Update NVS if needed

    //
    //  Note: It is important that this is only called 1 x per Second
    //
    updateSpeedDistRuntime();

    idleCount++;

    //
    // Enter Low Power mode if Idle (not moving) for required time
    //
    if( idleCount >= IDLE_TIME_LOW_POWER )
    {
/*
#ifdef DEBUG_SER_MON
      Serial.printf( "Power Save Mode On\n" );
#endif
*/

#ifdef LCD_DISP
      lcd.noBacklight();  // Turn off the backlight
#endif

#ifdef OLED_DISP
      oled.setPowerSave( ON );
#endif

    } // End of if()

    else
    {
#ifdef LCD_DISP
      lcd.backlight();    // Turn the backlight on
#endif

#ifdef OLED_DISP
      oled.setPowerSave( OFF );
#endif

      //
      // Format runtime in hours, minutes, seconds
      //
      int runtimeHours    =  runTimeSecs / 3600;
      int runtimeMinutes  = (runTimeSecs % 3600) / 60;
      int runtimeSeconds  =  runTimeSecs %   60;

      //
      // Get the day of the week
      //
      String dayOfWeek = getDayOfWeek( now.dayOfTheWeek() );

      //
      // Convert hour to 12-hour format
      //
      int hour12 = now.hour() % 12;
          hour12 = hour12 ? hour12 : 12; // Handle 0 hour case (midnight)
      String amPm = (now.hour() < 12) ? "AM" : "PM"; // Determine AM/PM

      //***********************************************************************
      // Update Display(s) with formatted data
      //***********************************************************************

      //-----------------------------------------------------------------------
      //  1st Line[0]: Time Display
      //-----------------------------------------------------------------------
#ifdef DEBUG_SER_MON
      Serial.printf( "%s %02d:%02d:%02d %s|", 
                  dayOfWeek.c_str(), 
                  hour12, now.minute(), 
                  now.second(), 
                  amPm.c_str());
#endif

#ifdef LCD_DISP
      lcd.setCursor(0, 0);
      lcd.printf( "  %s %02d:%02d:%02d %s ", 
                  dayOfWeek.c_str(), 
                  hour12, now.minute(), 
                  now.second(), 
                  amPm.c_str());
#endif

#ifdef OLED_DISP
      oled.clearBuffer();  // Clear the display buffer
      oled.setCursor(OLED_OFFSET + 0, 10);
      oled.printf( "  %s %02d:%02d:%02d %s ", 
                   dayOfWeek.c_str(), 
                   hour12, now.minute(), 
                   now.second(), 
                   amPm.c_str());
#endif

      //-----------------------------------------------------------------------
      //  2nd Line[1]: Runtime
      //-----------------------------------------------------------------------
#ifdef DEBUG_SER_MON
      Serial.printf( "Run: %02d:%02d:%02d|", 
                  runtimeHours, 
                  runtimeMinutes, 
                  runtimeSeconds);
#endif

#ifdef LCD_DISP
      lcd.setCursor(0, 1);
      lcd.printf( "Run:  %02d:%02d:%02d ", 
                  runtimeHours, 
                  runtimeMinutes, 
                  runtimeSeconds);
#endif

#ifdef OLED_DISP
      oled.setCursor(OLED_OFFSET + 0, 20);
      oled.printf( "Run:  %02d:%02d:%02d ", 
                   runtimeHours, 
                   runtimeMinutes, 
                   runtimeSeconds);
#endif

      //-----------------------------------------------------------------------
      //  3nd Line[2]: Speed
      //-----------------------------------------------------------------------
#ifdef DEBUG_SER_MON
      Serial.printf( "%.2f MPH|%.2f Max|", 
                  currentSpeed,
                  maxSpeed);
#endif

#ifdef LCD_DISP
      lcd.setCursor(0, 2);
      lcd.printf( "%.2f MPH %.2f Max ", 
                  currentSpeed,
                  maxSpeed);
#endif

#ifdef OLED_DISP
      oled.setCursor(OLED_OFFSET + 0, 30);
      oled.printf( "%.2f MPH %.2f Max ", 
                   currentSpeed,
                   maxSpeed);
#endif

      //-----------------------------------------------------------------------
      //  4th Line[3]: Distance
      //-----------------------------------------------------------------------
#ifdef DEBUG_SER_MON
      Serial.printf( "%.3f Miles\n", 
                  totalDistance);
#endif

#ifdef LCD_DISP
      lcd.setCursor(0, 3);
      lcd.printf( "%.3f Miles ", 
                  totalDistance);
#endif

#ifdef OLED_DISP
      oled.setCursor(OLED_OFFSET + 0, 40);
      oled.printf( "%.3f Miles ", 
                   totalDistance);

#endif

      //-----------------------------------------------------------------------
      //  5th Line[4]: Battery Voltage
      //----------------------------------------------------------------------- 
      // Read the raw ADC value from the voltage divider
      int rawVoltage = analogRead(voltagePin);
      // Convert the raw ADC value to actual battery voltage
      float batteryVoltage =  ((rawVoltage * adcVoltage) / adcResolution)
                              * ((R1 + R2 + R3) / R2);

#ifdef OLED_DISP
      oled.setCursor(OLED_OFFSET + 0, 50);
      oled.print("Battery: ");
      oled.print(batteryVoltage, 2);      // Display the voltage with 2 decimal places
      oled.print(" Volts ");

      oled.sendBuffer();  // Push all the content to the OLED screen
#endif

    } // End of else

  } // End of if()

} // End of loop()

//*****************************************************************************
// Hall Sensor ISR - Counts Wheel Revolutions
//*****************************************************************************
//
//  Using ISR to monitor Wheel Revolutions, it keeps rotations from being
//  missed, which could occure in a polled environment.
//
//  Function Prototype above is:
//    void IRAM_ATTR hallISR( void );
//
//  IRAM_ATTR --> function should be stored in 
//  IRAM (Internal RAM) rather than Flash
//  Used for:
//    - Faster Access
//    - Interrupt Safety
//
//*****************************************************************************
void /*IRAM_ATTR*/ hallISR()  // using IRAM_ATTR here and in prototype causes
                              // Compiler Warning Error...
{
  unsigned long currentTime = millis(); // Get the current time

  if (currentTime - lastDebounceTime > debounceDelay) 
  {
    revCount++; // Increment the rotation count
    idleCount = 0;
    lastDebounceTime = currentTime; // Update the debounce time
  
  } // End of if()

} // End of hallISR()

//*****************************************************************************
//  Save Time Zone Offset to NVS (only write if changed)
//*****************************************************************************
void saveTimeZoneOffset( int offset ) 
{
  preferences.begin( "my-app", false );   // Open NVS storage 
                                          // with namespace "my-app"
  int currentOffset = preferences.getInt( "tz_offset",  // Load current offset
                                          DEFAULT_TIME_ZONE_OFFSET);

  if( currentOffset != offset ) 
  {
    preferences.putInt("tz_offset", offset);  // Save the time zone offset 
                                              // only if changed

#ifdef DEBUG_SER_MON
    Serial.printf("Save: TZ Offset = %i\n", offset);
#endif

  } // End of if() 
  
  else 
  {
#ifdef DEBUG_SER_MON
    Serial.println("No change in TZ Offset.      Write skipped.");
#endif

  } // End of else

  preferences.end();  // Close NVS

} // End of saveTimeZoneOffset()

//*****************************************************************************
//  Load Time Zone Offset from NVS
//*****************************************************************************
int loadTimeZoneOffset( void ) 
{
  preferences.begin("my-app", true);                // Open NVS storage in 
                                                    //  read-only mode
  int offset = preferences.getInt("tz_offset", 0);  // Get the saved time zone
                                                    //  offset (default is 0)
  preferences.end();                                // Close NVS
  
#ifdef DEBUG_SER_MON
  Serial.printf("Load: TZ Offset = %i\n", offset);
#endif
  
  return offset;

} // End of loadTimeZoneOffset()

//*****************************************************************************
//  Save Wheel Diameter to NVS (only write if changed)
//*****************************************************************************
void saveWheelDiameter(float diameter) 
{
  preferences.begin("my-app", false);       // Open NVS storage with 
                                            // namespace "my-app"
  float currentDiameter = preferences.getFloat( "wheel_diameter", // Load current offset
                                                DEFAULT_WHEEL_DIAMETER);

//  if (fabs(currentDiameter - diameter) > 0.001)
  if( currentDiameter != diameter )
  {
    preferences.putFloat("wheel_diameter", diameter); // Save the time zone 
                                                      // offset only if changed

#ifdef DEBUG_SER_MON
    Serial.printf("Save: Wheel Diameter = %.2f\n", diameter);
#endif

  } // End of if() 
  
  else 
  {
#ifdef DEBUG_SER_MON
    Serial.println("No change in Wheel Diameter. Write skipped.");
#endif

  } // End of else

  preferences.end();  // Close NVS

} // End of saveWheelDiameter()

//*****************************************************************************
//  Load Wheel Diameter from NVS
//*****************************************************************************
float loadWheelDiameter( void ) 
{
  preferences.begin("my-app", true);                // Open NVS storage in 
                                                    //  read-only mode
  float diameter = preferences.getFloat("wheel_diameter", 0);  // Get the 
                                                    // saved Wheel Diameter
                                                    // (default is 0)
  preferences.end();                                // Close NVS
  
#ifdef DEBUG_SER_MON
  Serial.printf("Load: Wheel Diameter = %.2f\n", diameter);
#endif
  
  return diameter;

} // End of loadTimeZoneOffset()

//*****************************************************************************
//  Save Maximum Speed to NVS (only write if changed)
//*****************************************************************************
void saveMaxSpeed(float maxSpeed) 
{
  preferences.begin("my-app", false); // Open NVS storage with 
                                      // namespace "my-app"
  float currentMaxSpeed = preferences.getFloat("maxSpeed", 0.0);  // Load 
                                      // current max speed

  if (currentMaxSpeed != maxSpeed) 
  {
    preferences.putFloat("maxSpeed", maxSpeed); // Save only if the speed 
                                                // has changed

#ifdef DEBUG_SER_MON
    Serial.printf("Save: Max Speed = %.1f\n", maxSpeed);
#endif
  
  } // End of if()
  
  else 
  {
#ifdef DEBUG_SER_MON
    Serial.println("No change in Max Speed.      Write skipped.");
#endif
  
  } // End of else

  preferences.end();  // Close NVS

} // End of saveMaxSpeed()

//*****************************************************************************
//  Load Maximum Speed from NVS
//*****************************************************************************
float loadMaxSpeed( void ) 
{
  preferences.begin("my-app", true);  // Namespace "my-app" and true for 
                                      // read-only mode
  float maxSpeed = preferences.getFloat("maxSpeed", 0.0); // Default to 0.0 
                                                          //  if not found
  preferences.end(); // Always end after using preferences: Close NVS
  
#ifdef DEBUG_SER_MON
  Serial.printf("Load: Max Speed = %.1f\n", maxSpeed);
#endif
  
  return maxSpeed;

} // End of loadMaxSpeed()

//*****************************************************************************
//  Save Run Time to NVS (only write if changed)
//*****************************************************************************
void saveRunTimeSecs(unsigned long runTime)
{
  preferences.begin("my-app", false); // Open NVS storage with 
                                      // namespace "my-app"
  unsigned long currentRunTime = preferences.getULong("runTime", 0);  // Load 
                                      // current run time

  if (currentRunTime != runTime) 
  {
    preferences.putULong("runTime", runTime); // Save only if the run time 
                                              // has changed

#ifdef DEBUG_SER_MON
    Serial.printf("Save: Run Time = %u\n", runTime);
#endif
  
  } // End of if() 
  
  else 
  {
#ifdef DEBUG_SER_MON
    Serial.println("No change in Run Time.       Write skipped.");
#endif
  
  } // End of else

  preferences.end();  // Close NVS

} // End of saveRunTimeSecs()

//*****************************************************************************
//  Load Run Time from NVS
//*****************************************************************************
unsigned long loadRunTimeSecs( void ) 
{
    preferences.begin("my-app", true);  // Open NVS storage in read-only mode 
                                        // with namespace "my-app"
    unsigned long runTime = preferences.getULong("runTime", 0);  // Get the run 
                                        // time, default to 0 if not found
    preferences.end();  // Close Preferences

#ifdef DEBUG_SER_MON
    Serial.printf("Load: Run Time = %lu seconds\n", runTime);
#endif

    return runTime;  // Return the loaded value

} // End of loadRunTimeSecs()

//*****************************************************************************
//  Save Total Distance to NVS (only write if changed)
//*****************************************************************************
void saveTotalDistance(float distance) 
{
  preferences.begin("my-app", false);  // Open NVS storage with namespace "my-app"
  float currentDistance = preferences.getFloat( "distance", 
                                                0.0);  // Load current distance

  if (currentDistance != distance) 
  {
    preferences.putFloat("distance", distance); // Save only if the 
                                                // distance has changed

#ifdef DEBUG_SER_MON
    Serial.printf("Save: Total Distance = %.3f\n", distance);
#endif

  } // End if if()
  
  else 
  {
#ifdef DEBUG_SER_MON
    Serial.println("No change in Total Distance. Write skipped.");
#endif

  } // End of else

  preferences.end();  // Close NVS

} // End of saveTotalDistance()

//*****************************************************************************
//  Load Total Distance from NVS
//*****************************************************************************
float loadTotalDistance( void )
{
    preferences.begin("my-app", true);
    float distance = preferences.getFloat("distance", 0.0);
    preferences.end();

#ifdef DEBUG_SER_MON
    Serial.printf("Load: Total Distance = %.3f\n", distance);
#endif

    return distance;

} // End of loadTotalDistance()

//*****************************************************************************
//  Save NVS [if changed] every 60 seconds
//*****************************************************************************
void updateNVS( void )
{
  static int secondsCount = 0;

  secondsCount++;

  if ( secondsCount >= 60 )
  {
    secondsCount = 0;

    saveMaxSpeed( maxSpeed );           // Maximun Speed
    saveRunTimeSecs( runTimeSecs );     // Run Time Seconds
    saveTotalDistance( totalDistance);  // Total Distance

  } // End of if()

} // End of updateNVS()

//*****************************************************************************
//  Update Speed, Distance and Runtime Function
//*****************************************************************************
void updateSpeedDistRuntime( void ) 
{
  //
  // Constants for Speed and Distance calculations
  //
  const float inchPerMile       = 5280.0 * 12.0;  // Inches per Mile
  const float secondsPerHour    =   60.0 * 60.0;  // Seconds per Hour

  float wheelCircInMiles  = (wheelDiameter * PI) / inchPerMile;

  //
  //  Current Speed during last Second
  //
  currentSpeed =  ( revCount *            // # of Revolutions of the wheel x
                    wheelCircInMiles) /   // Circumference in Miles /
                    (1.0) *               //  1 Second x
                    secondsPerHour;       // Seconds in an Hour

  //
  //  Adjust Max Speed if needed
  //
  if( currentSpeed > maxSpeed )
  {
    maxSpeed = currentSpeed;

    //
    //  Save Max Speed in NVS
    //
    //  This shouldn't be a problem as 
    //  reaching a Max Speed will not be a frequent occurance.
    //
    saveMaxSpeed( maxSpeed );

  } // End of if()

  //
  // Update Runtime ONLY IF moving!!!
  //
  if( revCount != 0)
  {
    runTimeSecs++;

  } // End of if()

  //
  // Accumulate total distance traveled
  //
  totalDistance += (revCount * wheelCircInMiles);

  //
  //  Prepair for next second processing
  //
  revCount = 0;         // Reset revolution count

} // End of updateSpeedDistRuntime()

//*****************************************************************************
//  Get Day of Week Function
//*****************************************************************************
String getDayOfWeek( uint8_t wday ) 
{
  switch(wday) 
  {
    case 0: return "Sun";
    case 1: return "Mon";
    case 2: return "Tue";
    case 3: return "Wed";
    case 4: return "Thu";
    case 5: return "Fri";
    case 6: return "Sat";
    default: return "???";
  
  } // End of switch()

} // End of getDayOfWeek()
