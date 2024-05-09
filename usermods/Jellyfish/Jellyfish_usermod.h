#pragma once

#include "wled.h"
#include "TMC2209.h"
#include "Temperature_LM75_Derived.h"
#include <SparkFun_VEML7700_Arduino_Library.h>


/*
 * Usermods allow you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * 
 * This is an example for a v2 usermod.
 * v2 usermods are class inheritance based and can (but don't have to) implement more functions, each of them is shown in this example.
 * Multiple v2 usermods can be added to one compilation easily.
 * 
 * Creating a usermod:
 * This file serves as an example. If you want to create a usermod, it is recommended to use usermod_v2_empty.h from the usermods folder as a template.
 * Please remember to rename the class and file to a descriptive name.
 * You may also use multiple .h and .cpp files.
 * 
 * Using a usermod:
 * 1. Copy the usermod into the sketch folder (same folder as wled00.ino)
 * 2. Register the usermod by adding #include "usermod_filename.h" in the top and registerUsermod(new MyUsermodClass()) in the bottom of usermods_list.cpp
 */



#define JELLYFISH_DEBOUNCE_THRESHOLD      50 // only consider button input of at least 50ms as valid (debouncing)
#define JELLYFISH_LONG_PRESS             600 // long press if button is released after held for at least 600ms
#define JELLYFISH_DOUBLE_PRESS           350 // double press if another press within 350ms after a short press
#define JELLYFISH_LONG_REPEATED_ACTION   300 // how often a repeated action (e.g. dimming) is fired on long press on button IDs >0
#define JELLYFISH_LONG_AP               5000 // how long button 0 needs to be held to activate WLED-AP
#define JELLYFISH_LONG_FACTORY_RESET   10000 // how long button 0 needs to be held to trigger a factory reset
static const char _mqtt_topic_button[] PROGMEM = "%s/button/%d";  // optimize flash usage


class JellyfishUsermod : public Usermod {

  private:

    // Private class members. You can declare variables and functions only accessible to your usermod here

    Generic_LM75_12Bit tmp1075n;
    VEML7700 veml7700; // Create a VEML7700 object

    float lux = 0.0f;
    float temperature = 0.0f;

    bool enabled = false;
    bool initDone = false;
    unsigned long lastTime = 0;


    bool motion_enabled = false;

    bool tmp1075n_enabled = false;
    bool veml7700_enabled = false;



    const int32_t veloArray[21] = {52000,70000,90000,110000,95000,90000,85000,80000,75000,70000,65000,60000,55000,54000,54000,54000,52000,52000,52000,52000,52000};
    const int32_t veloRampSpeed = 5000;   
    unsigned long currentTime;
    unsigned long loopTime;
    int step = 0;
    int triggered = 0;

    const int step_offset = 15;
    int step_offset_cnt = 0;




    int8_t RXD2Pin = 18; // RX pin for UART2 on Cyanea Board
    int8_t TXD2Pin = 17; // TX pin for UART2 on Cyanea Board
    int8_t ENNPin = 8; // TMC2209 ENN pin (active low)
    int8_t HallSensPin = 7; // hall sensor trigger pin (active low)
    

    int32_t VELOCITY = 55000;   
    int32_t curr_velocity = 0;
    int32_t target_velocity = 0;
    int32_t velo_ramp_step = 0;
    int32_t velo_ramp_cur = 0;
    unsigned long lastTimeVelo = 0;    


    uint8_t RUN_CURRENT_PERCENT = 20;
    uint8_t PWM_gradient = 80;          // upper limit should be at 80 to 100, otherwise motor current and temperature are too high
    unsigned long Microsteps = 256;

    // Instantiate TMC2209
    TMC2209 stepper_driver;

    // strings to reduce flash memory usage (used more than twice)
    static const char _name[];
    static const char _enabled[];
    static const char _motion[];    
    static const char _VELOCITY[];
    static const char _runcurrent[];
    static const char _pwmgradient[];
    static const char _Microsteps[];





    // any private methods should go here (non-inline method should be defined out of class)
    void publishMqtt(const char* state, bool retain = false); // example for publishing MQTT message


  public:

    // non WLED related methods, may be used for data exchange between usermods (non-inline methods should be defined out of class)

    /**
     * Enable/Disable the usermod
     */
    inline void enable(bool enable) { enabled = enable; }

    /**
     * Get usermod enabled/disabled state
     */
    inline bool isEnabled() { return enabled; }

    // in such case add the following to another usermod:
    //  in private vars:
    //   #ifdef USERMOD_EXAMPLE
    //   MyExampleUsermod* UM;
    //   #endif
    //  in setup()
    //   #ifdef USERMOD_EXAMPLE
    //   UM = (MyExampleUsermod*) usermods.lookup(USERMOD_ID_EXAMPLE);
    //   #endif
    //  somewhere in loop() or other member method
    //   #ifdef USERMOD_EXAMPLE
    //   if (UM != nullptr) isExampleEnabled = UM->isEnabled();
    //   if (!isExampleEnabled) UM->enable(true);
    //   #endif






    void enableMotion()
    {
      motion_enabled = true;
      stepper_driver.enable();
      stepper_driver.moveAtVelocity(curr_velocity);

      // enable TMC2209 output
      digitalWrite(ENNPin, LOW);
    }

    void disableMotion()
    {
      motion_enabled = false;
      stepper_driver.disable();
      // disable TMC2209 output
      digitalWrite(ENNPin, HIGH);
    }    




    void MotionRampUp()
    {
      lastTimeVelo = millis();

      target_velocity = VELOCITY;
      velo_ramp_cur = veloRampSpeed;
      curr_velocity = veloRampSpeed;
      enableMotion();
    }

    void MotionRampDown()
    {
      lastTimeVelo = millis();

      target_velocity = 0;
      velo_ramp_cur =- veloRampSpeed;
      curr_velocity -= veloRampSpeed;

      if(curr_velocity <= 0)
      {
        disableMotion();
      }
    }

    void MotionRampUpdate()
    {
      if(motion_enabled)
      {

        if(curr_velocity != target_velocity)
        {

          if (millis() - lastTimeVelo > 100) 
          {
              lastTimeVelo = millis();

              curr_velocity += velo_ramp_cur;
              
              if(velo_ramp_cur < 0)
              {
                  if(curr_velocity <= 0)
                  {
                    curr_velocity = 0;
                    velo_ramp_cur = 0;
                  }
              }

              if(velo_ramp_cur > 0)
              {
                  if(curr_velocity >= target_velocity)
                  {
                    curr_velocity = target_velocity;
                    velo_ramp_cur = 0;
                  }
              }

              stepper_driver.moveAtVelocity(curr_velocity);

              if(curr_velocity == 0)
              {
                // switch off motor driver
                disableMotion();
              }
          }
        }
      }
    }









    // methods called by WLED (can be inlined as they are called only once but if you call them explicitly define them out of class)

    /*
     * setup() is called once at boot. WiFi is not yet connected at this point.
     * readFromConfig() is called prior to setup()
     * You can use it to initialize variables, sensors or similar.
     */
    void setup() {
      // do your set-up here
      Serial.println("Hello from Jellyfish usermod!");

      if (!enabled) return;

      PinManagerPinType pins[4] = { { RXD2Pin, false }, { TXD2Pin, true }, { ENNPin, true } };
      if (!pinManager.allocateMultiplePins(pins, 3, PinOwner::UM_JELLYFISH)) {

        RXD2Pin = TXD2Pin = ENNPin = -1;
        enabled = false;
        return;
      }

      pinMode(RXD2Pin, INPUT);
      pinMode(TXD2Pin, OUTPUT);
      pinMode(ENNPin, OUTPUT);
      pinMode(HallSensPin, INPUT);  // hall sensor
      // disable TMC2209 output
      digitalWrite(ENNPin, HIGH);


      //Serial2.begin(115200, SERIAL_8N1, RXD2Pin, TXD2Pin);

      stepper_driver.setup(Serial2, 115200L, TMC2209::SERIAL_ADDRESS_0, RXD2Pin, TXD2Pin);
      stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
      stepper_driver.setHoldCurrent(0);
      //stepper_driver.useInternalSenseResistors();
      //stepper_driver.disableStealthChop();
      stepper_driver.disableAutomaticGradientAdaptation();
      stepper_driver.disableAutomaticCurrentScaling();
      //stepper_driver.setPwmGradient(150);   // 250 -> 370mA, high torque, 200 -> 70mA barely enough torque
      stepper_driver.setPwmGradient(PWM_gradient);   // 250 -> 370mA, high torque, 200 -> 70mA barely enough torque
      stepper_driver.setPwmOffset(0);

      stepper_driver.setMicrostepsPerStep(Microsteps);
      //stepper_driver.enableAnalogCurrentScaling();

      //stepper_driver.enableCoolStep();


      if(motion_enabled)
        enableMotion();


    if (i2c_scl<0 || i2c_sda<0) 
    {  
      veml7700_enabled = false;
      tmp1075n_enabled = false;
    }
    else
    {
      veml7700_enabled = veml7700.begin();
      tmp1075n_enabled = true;
    }



      initDone = true;
    }


    /*
     * connected() is called every time the WiFi is (re)connected
     * Use it to initialize network interfaces
     */
    void connected() {
      //Serial.println("Connected to WiFi!");


    }


    /*
     * loop() is called continuously. Here you can check for events, read sensors, etc.
     * 
     * Tips:
     * 1. You can use "if (WLED_CONNECTED)" to check for a successful network connection.
     *    Additionally, "if (WLED_MQTT_CONNECTED)" is available to check for a connection to an MQTT broker.
     * 
     * 2. Try to avoid using the delay() function. NEVER use delays longer than 10 milliseconds.
     *    Instead, use a timer check as shown here.
     */
    void loop() {
      // if usermod is disabled or called during strip updating just exit
      // NOTE: on very long strips strip.isUpdating() may always return true so update accordingly
      if (!enabled || strip.isUpdating()) return;

      // do your magic here

      MotionRampUpdate();   // update motor speed if required

      if (millis() - lastTime > 5000) 
      {
        //Serial.println("I'm alive!");
        lastTime = millis();


        if(tmp1075n_enabled)
        {
          Serial.print("Temperature = ");
          temperature = tmp1075n.readTemperatureC();
          Serial.print(temperature);
          Serial.println(" C");
        }

        if(veml7700_enabled)
        {
          Serial.print("ambient light = ");
          lux = veml7700.getLux();
          Serial.print(lux);
          Serial.println(" lux");
        }



  // Serial.println("*************************");
  // Serial.println("getStatus()");
  // TMC2209::Status status = stepper_driver.getStatus();
  // Serial.print("status.over_temperature_warning = ");
  // Serial.println(status.over_temperature_warning);
  // Serial.print("status.over_temperature_shutdown = ");
  // Serial.println(status.over_temperature_shutdown);
  // Serial.print("status.short_to_ground_a = ");
  // Serial.println(status.short_to_ground_a);
  // Serial.print("status.short_to_ground_b = ");
  // Serial.println(status.short_to_ground_b);
  // Serial.print("status.low_side_short_a = ");
  // Serial.println(status.low_side_short_a);
  // Serial.print("status.low_side_short_b = ");
  // Serial.println(status.low_side_short_b);
  // Serial.print("status.open_load_a = ");
  // Serial.println(status.open_load_a);
  // Serial.print("status.open_load_b = ");
  // Serial.println(status.open_load_b);
  // Serial.print("status.over_temperature_120c = ");
  // Serial.println(status.over_temperature_120c);
  // Serial.print("status.over_temperature_143c = ");
  // Serial.println(status.over_temperature_143c);
  // Serial.print("status.over_temperature_150c = ");
  // Serial.println(status.over_temperature_150c);
  // Serial.print("status.over_temperature_157c = ");
  // Serial.println(status.over_temperature_157c);
  // Serial.print("status.current_scaling = ");
  // Serial.println(status.current_scaling);
  // Serial.print("status.stealth_chop_mode = ");
  // Serial.println(status.stealth_chop_mode);
  // Serial.print("status.standstill = ");
  // Serial.println(status.standstill);
  // Serial.println("*************************");
  // Serial.println();



  // // check for hall sensor trigger
  // if(!digitalRead(7))
  // {
  //   digitalWrite(1, LOW);

  //   if(!triggered)
  //   {
  //     triggered = 1;
  //     //step = 0; //step_offset;
  //     step_offset_cnt = step_offset;
  //   }
  // }
  // else
  // {
  //   digitalWrite(1, HIGH);
  // }





  //   currentTime = millis(); // get the current elapsed time

  //   if (currentTime >= (loopTime + 50)) // 100ms since last check 
  //   {

  //     stepper_driver.moveAtVelocity(veloArray[step]);
  //     step++;

  //     if(step >= 21)
  //     {
  //       step = 20;
  //       triggered = 0;
  //     }

  //     if(step_offset_cnt)
  //     {
  //       step_offset_cnt--;
  //       if(!step_offset_cnt)
  //       {
  //         step = 0;
  //       }
  //     }


  //     loopTime = currentTime; // Updates loopTime
  //   }






      }
    }


    /*
     * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
     * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
     * Below it is shown how this could be used for e.g. a light sensor
     */
    void addToJsonInfo(JsonObject& root)
    {
      // if "u" object does not exist yet wee need to create it
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      //this code adds "u":{"ExampleUsermod":[20," lux"]} to the info object
      //int reading = 20;
      //JsonArray lightArr = user.createNestedArray(FPSTR(_name))); //name
      //lightArr.add(reading); //value
      //lightArr.add(F(" lux")); //unit

      // if you are implementing a sensor usermod, you may publish sensor data
      //JsonObject sensor = root[F("sensor")];
      //if (sensor.isNull()) sensor = root.createNestedObject(F("sensor"));
      //temp = sensor.createNestedArray(F("light"));
      //temp.add(reading);
      //temp.add(F("lux"));


      //this code adds "u":{"ExampleUsermod":[20," lux"]} to the info object
      //int reading = 20;
      JsonArray lightArr = user.createNestedArray(F("Luminance")); //name
      lightArr.add(lux); //value
      lightArr.add(F(" lux")); //unit

      JsonArray tempArr = user.createNestedArray(F("Temperature")); //name
      tempArr.add(temperature); //value
      tempArr.add(F(" °C")); //unit


    // JsonArray lux_json = user.createNestedArray(F("Luminance"));
    // if (!enabled) {
    //   lux_json.add(F("disabled"));
    // } else if (!sensorFound) {
    //     // if no sensor 
    //     lux_json.add(F("BH1750 "));
    //     lux_json.add(F("Not Found"));
    // } else if (!getLuminanceComplete) {
    //   // if we haven't read the sensor yet, let the user know
    //     // that we are still waiting for the first measurement
    //     lux_json.add((USERMOD_BH1750_FIRST_MEASUREMENT_AT - millis()) / 1000);
    //     lux_json.add(F(" sec until read"));
    //     return;
    // } else {
    //   lux_json.add(lastLux);
    //   lux_json.add(F(" lx"));
    // }



    }


    /*
     * addToJsonState() can be used to add custom entries to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
    void addToJsonState(JsonObject& root)
    {
      if (!initDone || !enabled) return;  // prevent crash on boot applyPreset()

      JsonObject usermod = root[FPSTR(_name)];
      if (usermod.isNull()) usermod = root.createNestedObject(FPSTR(_name));

      //usermod["user0"] = userVar0;

      JsonObject motorState = usermod.createNestedObject(FPSTR(_motion));
      motorState[FPSTR(_motion)] = motion_enabled ? false : true;      
    }


    /*
     * readFromJsonState() can be used to receive data clients send to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
    void readFromJsonState(JsonObject& root)
    {
      if (!initDone) return;  // prevent crash on boot applyPreset()

      bool en = false;

      Serial.print("readFromJsonState");

      JsonObject um = root[FPSTR(_name)];
      if (!um.isNull()) {
        // expect JSON usermod data in usermod name object: {"ExampleUsermod:{"user0":10}"}
        //userVar0 = usermod["motor"] | userVar0; //if "user0" key exists in JSON, update, else keep old value

        Serial.print("um is not null");

        if (um[FPSTR(_motion)].is<bool>()) {
          en = um[FPSTR(_motion)].as<bool>();
        } else {
          String str = um[FPSTR(_motion)]; // checkbox -> off or on
          en = (bool)(str!="off"); // off is guaranteed to be present
        }


        if(en == true)
        {
          //enableMotion();
          MotionRampUp();
          Serial.print("enableMotion");
        }
        else
        {
          //disableMotion();
          MotionRampDown();
          Serial.print("disableMotion");
        }

        // JsonObject motorState = usermod[F("motorState")];

        // if (motorState.isNull()) {
        //   return;
        // }
        // bool menable;
        // if (getJsonValue(motorState[F("motion")], menable)) {
        //   if(menable == true)
        //     enableMotion();
        //   else
        //     disableMotion();
        // }        
      }
      // you can as well check WLED state JSON keys
      //if (root["bri"] == 255) Serial.println(F("Don't burn down your garage!"));
    }




    // void addToJsonState(JsonObject& pwmState) const {
    //   pwmState[F("duty")] = duty_;
    // }

    // void readFromJsonState(JsonObject& pwmState) {
    //   if (pwmState.isNull()) {
    //     return;
    //   }
    //   float duty;
    //   if (getJsonValue(pwmState[F("duty")], duty)) {
    //     setDuty(duty);
    //   }
    // }

    // void addToJsonState(JsonObject& root) {
    //   JsonObject pwmStates = root.createNestedObject(PWM_STATE_NAME);
    //   for (int i = 0; i < USERMOD_PWM_OUTPUT_PINS; i++) {
    //     const PwmOutput& pwm = pwms_[i];
    //     if (!pwm.isEnabled())
    //       continue;
    //     char buffer[4];
    //     sprintf_P(buffer, PSTR("%d"), i);
    //     JsonObject pwmState = pwmStates.createNestedObject(buffer);
    //     pwm.addToJsonState(pwmState);
          //pwmState[F("duty")] = duty_;



    //   }
    // }

    // void readFromJsonState(JsonObject& root) {
    //   JsonObject pwmStates = root[PWM_STATE_NAME];
    //   if (pwmStates.isNull())
    //     return;

    //   for (int i = 0; i < USERMOD_PWM_OUTPUT_PINS; i++) {
    //     PwmOutput& pwm = pwms_[i];
    //     if (!pwm.isEnabled())
    //       continue;
    //     char buffer[4];
    //     sprintf_P(buffer, PSTR("%d"), i);
    //     JsonObject pwmState = pwmStates[buffer];
    //     pwm.readFromJsonState(pwmState);
    //   }
    // }













    /*
     * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
     * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
     * If you want to force saving the current state, use serializeConfig() in your loop().
     * 
     * CAUTION: serializeConfig() will initiate a filesystem write operation.
     * It might cause the LEDs to stutter and will cause flash wear if called too often.
     * Use it sparingly and always in the loop, never in network callbacks!
     * 
     * addToConfig() will make your settings editable through the Usermod Settings page automatically.
     *
     * Usermod Settings Overview:
     * - Numeric values are treated as floats in the browser.
     *   - If the numeric value entered into the browser contains a decimal point, it will be parsed as a C float
     *     before being returned to the Usermod.  The float data type has only 6-7 decimal digits of precision, and
     *     doubles are not supported, numbers will be rounded to the nearest float value when being parsed.
     *     The range accepted by the input field is +/- 1.175494351e-38 to +/- 3.402823466e+38.
     *   - If the numeric value entered into the browser doesn't contain a decimal point, it will be parsed as a
     *     C int32_t (range: -2147483648 to 2147483647) before being returned to the usermod.
     *     Overflows or underflows are truncated to the max/min value for an int32_t, and again truncated to the type
     *     used in the Usermod when reading the value from ArduinoJson.
     * - Pin values can be treated differently from an integer value by using the key name "pin"
     *   - "pin" can contain a single or array of integer values
     *   - On the Usermod Settings page there is simple checking for pin conflicts and warnings for special pins
     *     - Red color indicates a conflict.  Yellow color indicates a pin with a warning (e.g. an input-only pin)
     *   - Tip: use int8_t to store the pin value in the Usermod, so a -1 value (pin not set) can be used
     *
     * See usermod_v2_auto_save.h for an example that saves Flash space by reusing ArduinoJson key name strings
     * 
     * If you need a dedicated settings page with custom layout for your Usermod, that takes a lot more work.  
     * You will have to add the setting to the HTML, xml.cpp and set.cpp manually.
     * See the WLED Soundreactive fork (code and wiki) for reference.  https://github.com/atuline/WLED
     * 
     * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
     */
    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      //save these vars persistently whenever settings are saved
      top[FPSTR(_VELOCITY)]    = VELOCITY;
      top[FPSTR(_runcurrent)] = RUN_CURRENT_PERCENT;
      top[FPSTR(_pwmgradient)] = PWM_gradient;
      top[FPSTR(_Microsteps)] = Microsteps;
      DEBUG_PRINTLN(F("Autosave config saved."));
    }


    /*
     * readFromConfig() can be used to read back the custom settings you added with addToConfig().
     * This is called by WLED when settings are loaded (currently this only happens immediately after boot, or after saving on the Usermod Settings page)
     * 
     * readFromConfig() is called BEFORE setup(). This means you can use your persistent values in setup() (e.g. pin assignments, buffer sizes),
     * but also that if you want to write persistent values to a dynamic buffer, you'd need to allocate it here instead of in setup.
     * If you don't know what that is, don't fret. It most likely doesn't affect your use case :)
     * 
     * Return true in case the config values returned from Usermod Settings were complete, or false if you'd like WLED to save your defaults to disk (so any missing values are editable in Usermod Settings)
     * 
     * getJsonValue() returns false if the value is missing, or copies the value into the variable provided and returns true if the value is present
     * The configComplete variable is true only if the "exampleUsermod" object and all values are present.  If any values are missing, WLED will know to call addToConfig() to save them
     * 
     * This function is guaranteed to be called on boot, but could also be called every time settings are updated
     */
    bool readFromConfig(JsonObject& root)
    {
      // default settings values could be set here (or below using the 3-argument getJsonValue()) instead of in the class definition or constructor
      // setting them inside readFromConfig() is slightly more robust, handling the rare but plausible use case of single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)

      JsonObject top = root[FPSTR(_name)];

      bool configComplete = !top.isNull();

      enabled           = top[FPSTR(_enabled)] | enabled;
      VELOCITY = top[FPSTR(_VELOCITY)] | VELOCITY;
      RUN_CURRENT_PERCENT    = top[FPSTR(_runcurrent)] | RUN_CURRENT_PERCENT;
      //minPWMValuePct    = (uint8_t) min(100,max(0,(int)minPWMValuePct)); // bounds checking
      PWM_gradient    = top[FPSTR(_pwmgradient)] | PWM_gradient;
      //maxPWMValuePct    = (uint8_t) min(100,max((int)minPWMValuePct,(int)maxPWMValuePct)); // bounds checking
      Microsteps = top[FPSTR(_Microsteps)] | Microsteps;
      //numberOfInterrupsInOneSingleRotation = (uint8_t) max(1,(int)numberOfInterrupsInOneSingleRotation); // bounds checking

      return !top[FPSTR(_Microsteps)].isNull();
    }


    /*
     * appendConfigData() is called when user enters usermod settings page
     * it may add additional metadata for certain entry fields (adding drop down is possible)
     * be careful not to add too much as oappend() buffer is limited to 3k
     */
    void appendConfigData()
    {
      oappend(SET_F("addInfo('")); oappend(String(FPSTR(_name)).c_str()); oappend(SET_F(":great")); oappend(SET_F("',1,'<i>(this is a great config value)</i>');"));
      oappend(SET_F("addInfo('")); oappend(String(FPSTR(_name)).c_str()); oappend(SET_F(":testString")); oappend(SET_F("',1,'enter any string you want');"));
      oappend(SET_F("dd=addDropdown('")); oappend(String(FPSTR(_name)).c_str()); oappend(SET_F("','testInt');"));
      oappend(SET_F("addOption(dd,'Nothing',0);"));
      oappend(SET_F("addOption(dd,'Everything',42);"));
    }


    /*
     * handleOverlayDraw() is called just before every show() (LED strip update frame) after effects have set the colors.
     * Use this to blank out some LEDs or set them to a different color regardless of the set effect mode.
     * Commonly used for custom clocks (Cronixie, 7 segment)
     */
    void handleOverlayDraw()
    {
      //strip.setPixelColor(0, RGBW32(0,0,0,0)) // set the first pixel to black
    }



    /*
     * Custom key handlers to overide common WLED button behavior
     */
    void handleSwitchJellyfish(uint8_t b)
    {
      // isButtonPressed() handles inverted/noninverted logic
      if (buttonPressedBefore[b] != isButtonPressed(b)) {
        buttonPressedTime[b] = millis();
        buttonPressedBefore[b] = !buttonPressedBefore[b];
      }

      if (buttonLongPressed[b] == buttonPressedBefore[b]) return;

      if (millis() - buttonPressedTime[b] > JELLYFISH_DEBOUNCE_THRESHOLD) { //fire edge event only after 50ms without change (debounce)
        if (!buttonPressedBefore[b]) { // on -> off
          if (macroButton[b]) applyPreset(macroButton[b], CALL_MODE_BUTTON_PRESET);
          else { //turn on
            if (!bri) {toggleOnOff(); stateUpdated(CALL_MODE_BUTTON);}
          }
        } else {  // off -> on
          if (macroLongPress[b]) applyPreset(macroLongPress[b], CALL_MODE_BUTTON_PRESET);
          else { //turn off
            if (bri) {toggleOnOff(); stateUpdated(CALL_MODE_BUTTON);}
          }
        }

    #ifndef WLED_DISABLE_MQTT
        // publish MQTT message
        if (buttonPublishMqtt && WLED_MQTT_CONNECTED) {
          char subuf[64];
          if (buttonType[b] == BTN_TYPE_PIR_SENSOR) sprintf_P(subuf, PSTR("%s/motion/%d"), mqttDeviceTopic, (int)b);
          else sprintf_P(subuf, _mqtt_topic_button, mqttDeviceTopic, (int)b);
          mqtt->publish(subuf, 0, false, !buttonPressedBefore[b] ? "off" : "on");
        }
    #endif

        buttonLongPressed[b] = buttonPressedBefore[b]; //save the last "long term" switch state
      }
    }

    void handleButtonJellyfish()
    {
      static unsigned long lastRead = 0UL;
      static unsigned long lastRun = 0UL;
      unsigned long now = millis();

      if (strip.isUpdating() && (now - lastRun < 400)) return; // don't interfere with strip update (unless strip is updating continuously, e.g. very long strips)
      lastRun = now;

      for (uint8_t b=0; b<WLED_MAX_BUTTONS; b++) {
 
        if (btnPin[b]<0 || buttonType[b] == BTN_TYPE_NONE) continue;
 
        // button is not momentary, but switch. This is only suitable on pins whose on-boot state does not matter (NOT gpio0)
        if (buttonType[b] == BTN_TYPE_SWITCH || buttonType[b] == BTN_TYPE_PIR_SENSOR) {
 //         handleSwitch(b);
          continue;
        }

        // momentary button logic
        if (isButtonPressed(b)) { // pressed

          // if all macros are the same, fire action immediately on rising edge
          if (macroButton[b] && macroButton[b] == macroLongPress[b] && macroButton[b] == macroDoublePress[b]) {
            if (!buttonPressedBefore[b])
              shortPressAction(b);
            buttonPressedBefore[b] = true;
            buttonPressedTime[b] = now; // continually update (for debouncing to work in release handler)
            continue;
          }

          if (!buttonPressedBefore[b]) buttonPressedTime[b] = now;
          buttonPressedBefore[b] = true;

          if (now - buttonPressedTime[b] > JELLYFISH_LONG_PRESS) { //long press
            if (!buttonLongPressed[b]) longPressAction(b);
            else if (b) { //repeatable action (~3 times per s) on button > 0
              longPressAction(b);
              buttonPressedTime[b] = now - JELLYFISH_LONG_REPEATED_ACTION; //333ms
            }
            buttonLongPressed[b] = true;
          }

        } else if (!isButtonPressed(b) && buttonPressedBefore[b]) { //released
          long dur = now - buttonPressedTime[b];

          // released after rising-edge short press action
          if (macroButton[b] && macroButton[b] == macroLongPress[b] && macroButton[b] == macroDoublePress[b]) {
            if (dur > JELLYFISH_DEBOUNCE_THRESHOLD) buttonPressedBefore[b] = false; // debounce, blocks button for 50 ms once it has been released
            continue;
          }

          if (dur < JELLYFISH_DEBOUNCE_THRESHOLD) {buttonPressedBefore[b] = false; continue;} // too short "press", debounce
          bool doublePress = buttonWaitTime[b]; //did we have a short press before?
          buttonWaitTime[b] = 0;

          if (b == 0 && dur > JELLYFISH_LONG_AP) { // long press on button 0 (when released)
            if (dur > JELLYFISH_LONG_FACTORY_RESET) { // factory reset if pressed > 10 seconds
              WLED_FS.format();
              #ifdef WLED_ADD_EEPROM_SUPPORT
              clearEEPROM();
              #endif
              doReboot = true;
            } else {
              WLED::instance().initAP(true);
            }
          } else if (!buttonLongPressed[b]) { //short press
            //NOTE: this interferes with double click handling in usermods so usermod needs to implement full button handling
            if (b != 1 && !macroDoublePress[b]) { //don't wait for double press on buttons without a default action if no double press macro set
              shortPressAction(b);
            } else { //double press if less than 350 ms between current press and previous short press release (buttonWaitTime!=0)
              if (doublePress) {
                doublePressAction(b);
              } else {
                buttonWaitTime[b] = now;
              }
            }
          }
          buttonPressedBefore[b] = false;
          buttonLongPressed[b] = false;
        }

        //if 350ms elapsed since last short press release it is a short press
        if (buttonWaitTime[b] && now - buttonWaitTime[b] > JELLYFISH_DOUBLE_PRESS && !buttonPressedBefore[b]) {
          buttonWaitTime[b] = 0;
          shortPressAction(b);
        }
      }
    }














    /**
     * handleButton() can be used to override default button behaviour. Returning true
     * will prevent button working in a default way.
     * Replicating button.cpp
     */
    bool handleButton(uint8_t b) {
      yield();
      // ignore certain button types as they may have other consequences
      if (!enabled
       || buttonType[b] == BTN_TYPE_NONE
       || buttonType[b] == BTN_TYPE_RESERVED
       || buttonType[b] == BTN_TYPE_PIR_SENSOR
       || buttonType[b] == BTN_TYPE_ANALOG
       || buttonType[b] == BTN_TYPE_ANALOG_INVERTED) {
        return false;
      }

      bool handled = false;
      // do your button handling here
      return handled;
    }
  

#ifndef WLED_DISABLE_MQTT
    /**
     * handling of MQTT message
     * topic only contains stripped topic (part after /wled/MAC)
     */
    bool onMqttMessage(char* topic, char* payload) {
      // check if we received a command
      //if (strlen(topic) == 8 && strncmp_P(topic, PSTR("/command"), 8) == 0) {
      //  String action = payload;
      //  if (action == "on") {
      //    enabled = true;
      //    return true;
      //  } else if (action == "off") {
      //    enabled = false;
      //    return true;
      //  } else if (action == "toggle") {
      //    enabled = !enabled;
      //    return true;
      //  }
      //}
      return false;
    }

    /**
     * onMqttConnect() is called when MQTT connection is established
     */
    void onMqttConnect(bool sessionPresent) {
      // do any MQTT related initialisation here
      //publishMqtt("I am alive!");
    }
#endif


    /**
     * onStateChanged() is used to detect WLED state change
     * @mode parameter is CALL_MODE_... parameter used for notifications
     */
    void onStateChange(uint8_t mode) 
    {
      // do something if WLED state changed (color, brightness, effect, preset, etc)

        if (bri == 0)
        {
          MotionRampDown();
        }
        else
        {
          MotionRampUp();
        }

    }


    /*
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
    uint16_t getId()
    {
      return USERMOD_ID_JELLYFISH;
    }

   //More methods can be added in the future, this example will then be extended.
   //Your usermod will remain compatible as it does not need to implement all methods from the Usermod base class!
};


// add more strings here to reduce flash memory usage
const char JellyfishUsermod::_name[]    PROGMEM = "Jellyfish";

const char JellyfishUsermod::_enabled[] PROGMEM = "enabled";
const char JellyfishUsermod::_motion[] PROGMEM = "motion";

const char JellyfishUsermod::_VELOCITY[] PROGMEM = "Velocity";
const char JellyfishUsermod::_runcurrent[] PROGMEM = "Run current percent";
const char JellyfishUsermod::_pwmgradient[] PROGMEM = "PWM gradient";
const char JellyfishUsermod::_Microsteps[] PROGMEM = "Microsteps";





// implementation of non-inline member methods

void JellyfishUsermod::publishMqtt(const char* state, bool retain)
{
#ifndef WLED_DISABLE_MQTT
  //Check if MQTT Connected, otherwise it will crash the 8266
  if (WLED_MQTT_CONNECTED) {
    char subuf[64];
    strcpy(subuf, mqttDeviceTopic);
    strcat_P(subuf, PSTR("/example"));
    mqtt->publish(subuf, 0, retain, state);
  }
#endif
}
