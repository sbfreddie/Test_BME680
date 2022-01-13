/*! @file I2CDemo.ino

@section I2CDemo_intro_section Description

Example program for using I2C to set and read the Bosch BME680 sensor. The sensor measures
temperature, pressure and humidity and is described at
https://www.bosch-sensortec.com/bst/products/all_products/BME680. The datasheet is available from
Bosch at https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680_DS001-11.pdf \n\n

The most recent version of the BME680 library is available at https://github.com/Zanduino/BME680
and the documentation of the library as well as example programs are described in the project's wiki
pages located at https://github.com/Zanduino/BME680/wiki. \n\n

The BME680 is an extremely small physical package that is so tiny as to be impossible to solder at
home, hence it will be used as part of a third-party breakout board. There are several such boards
available at this time, for example \n
| Company  | Link       |
| -------  | ---------- |
| Sparkfun | https://www.sparkfun.com/products/14570 |
| BlueDot  | https://www.bluedot.space/sensor-boards/bme680/ |
| Adafruit | https://learn.adafruit.com/adafruit-BME680-humidity-barometric-pressure-temperature-sensor-breakout
|
\n\n

Bosch supplies sample software that runs on various platforms, including the Arduino family; this
can be downloaed at https://github.com/BoschSensortec/BSEC-Arduino-library . This software is part
of the Bosch "BSEC" (Bosch Sensortec Environmental Cluster) framework and somewhat bulky and
unwieldy for typical Arduino applications, hence the choice to make a more compact and rather less
abstract library.

This example program initializes the BME680 to use I2C for communications. The library does not
using floating point numbers to save on memory space and computation time. The values for
Temperature, Pressure and Humidity are returned in deci-units, e.g. a Temperature reading of "2731"
means "27.31" degrees Celsius. The display in the example program uses floating point for
demonstration purposes only.  Note that the temperature reading is generally higher than the ambient
temperature due to die and PCB temperature and self-heating of the element.\n\n

The pressure reading needs to be adjusted for altitude to get the adjusted pressure reading. There
are numerous sources on the internet for formulae converting from standard sea-level pressure to
altitude, see the data sheet for the BME180 on page 16 of
http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf. Rather than put a floating-point
function in the library which may not be used but which would use space, an example altitude
computation function has been added to this example program to show how it might be done.

@section I2CDemolicense License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU
General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version. This program is distributed in the hope that it will
be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should have
received a copy of the GNU General Public License along with this program.  If not, see
<http://www.gnu.org/licenses/>.

@section I2CDemoauthor Author

Written by Arnd <Arnd@Zanduino.Com> at https://www.github.com/SV-Zanshin

@section I2CDemoversions Changelog

| Version | Date       | Developer  | Comments
| ------- | ---------- | ---------- | ------------------------------------------------------------ |
| 1.0.3   | 2020-07-04 | SV-Zanshin | Issue #25 implement clang-formatting                         |
| 1.0.2   | 2020-05-09 | SV-Zanshin | Issue #8  clean up comments and code                         |
| 1.0.1   | 2019-01-30 | SV-Zanshin |           Removed old comments                               |
| 1.0.1   | 2019-01-26 | SV-Zanshin | Issue #3  convert documentation to Doxygen                   |
| 1.0.0b  | 2018-06-30 | SV-Zanshin |           Cloned from original BME280 program                |
*/

#include "Arduino.h"
#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
#include "Version.h"  // This has the most recent build version
#include <Wire.h>
#include <mcp9808.h>
#include <Streaming.h>

//#define findI2CAddresses 1	// First lets see if we can find all of our I2C devices.


/**************************************************************************************************
**                          Declare all program constants.                                       **
**************************************************************************************************/
const uint32_t SERIAL_SPEED{115200};  ///< Set the baud rate for Serial I/O

/**************************************************************************************************
**                               Instantiate classes                                             **
**************************************************************************************************/
BME680_Class BME680;  ///< Create an instance of the BME680 class.

// VALID ADDRESSES for MCP9808 are:
    // max 8 sensors on one bus, addresses can be:
        // 24 -> 31 Decimal == 0x18 -> 0x1F Hexidecimal
MCP9808 tempSensor(24);  // Create instance of the MCP9808 class.

/**************************************************************************************************
**                          Declare global variables.                                            **
**************************************************************************************************/
///< Forward function declaration with default value for sea level.
float altitude(const int32_t press, const float seaLevel = 1013.25);

/**************************************************************************************************
**                          Define Functions that will be used.                                  **
**************************************************************************************************/
float altitude(const int32_t press, const float seaLevel)
    {
        /*!****************************************************************************************
            @brief      This converts a pressure measurement into a height in meters
            @details    The corrected sea-level pressure can be passed into the function if it is known,
                        otherwise the standard atmospheric pressure of 1013.25hPa is used (see
                        https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
            @param[in]  press    Pressure reading from BME680
            @param[in]  seaLevel Sea-Level pressure in millibars
            @return     floating point altitude in meters.
        *******************************************************************************************/

        static float Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters. 
        return (Altitude);
    }  // of method altitude()
 
void displayData()
    {
        /*!****************************************************************************************
            @brief      Print the data from the MCP9808 sensor on the serial monitor.
            @details    
            @param[in]  
            @param[in]  
            @return     None.
        *******************************************************************************************/

        uint8_t status;

        if ( (status = tempSensor.isConnected()) == 1 )
            {
                float C = tempSensor.getTemperature();
                float F = ( (C  * 1.8) + 32.0);
                Serial << ("\n\nAmbient: ") << C << ("C  ") << F << ("F  ") << endl;

                C = tempSensor.getTupper();
                F = ( (C  * 1.8) + 32.0);
                Serial << ("Upper Limit: ") << C << ("C  ") << F << ("F  Alert: ") << _HEX(tempSensor.getStatus()) << endl;

                C = tempSensor.getTlower();
                F = ( (C  * 1.8) + 32.0);
                Serial << ("Lower Limit: ") << C << ("C  ") << F << ("F  Alert: ") << _HEX(tempSensor.getStatus()) << endl;

                C = tempSensor.getTcritical();
                F = ( (C  * 1.8) + 32.0);
                Serial << ("Critical Limit: ") << C << ("C  ") << F << ("F  Alert: ") << _HEX(tempSensor.getStatus()) << endl;

                Serial << ("Config 0x: ") << _HEX(tempSensor.getConfigRegister()) << endl;
                Serial << ("Resolution 0x: ") << _HEX(tempSensor.getResolution()) << endl;
                Serial << ("Mfr ID 0x: ") << _HEX(tempSensor.getManufacturerID()) << endl;
                Serial << ( "Device ID 0x:") << _HEX(tempSensor.getDeviceID()) << endl;
                Serial << ( "Device Rev 0x: ") << _HEX(tempSensor.getRevision()) << endl;
                Serial << ( "Temperature Offset Calibration: ") << (float)(tempSensor.getOffset()) << endl;
            }
        else
            {
                Serial << "Error reading sensor, status=" << status << endl;
            }
    }

#if defined(findI2CAddresses)
	// BUGBUG::Should put this in WireIMXRT.h
	#if defined(__IMXRT1062__)
		#define WIRE_IMPLEMENT_WIRE
		#define WIRE_IMPLEMENT_WIRE1
		#define WIRE_IMPLEMENT_WIRE2
	#endif
	typedef struct
		{
 			TwoWire *_wire;
  			const char * wire_name;
 			 IMXRT_LPI2C_t * const port;
		} wire_list_t;

	wire_list_t wireList[] =
		{
  			{&Wire, "Wire", &IMXRT_LPI2C1}
			#if defined(WIRE_IMPLEMENT_WIRE1)
  				, {&Wire1, "Wire1", &IMXRT_LPI2C3}
			#endif  
			#if defined(WIRE_IMPLEMENT_WIRE2)
  				, {&Wire2, "Wire2", &IMXRT_LPI2C4}
			#endif
			#if defined(WIRE_IMPLEMENT_WIRE3)
 				 , {&Wire3, "Wire3"}
			#endif
		};

	const uint8_t wirelist_count = sizeof(wireList) / sizeof(wireList[0]);

    void printKnownChips(byte address)
	    {
  		    // Is this list missing part numbers for chips you use?
  		    // Please suggest additions here:
  		    // https://github.com/PaulStoffregen/Wire/issues/new
  		    switch (address)
  			    {
   				    case 0x00: Serial.print(F("AS3935")); break;
				    case 0x01: Serial.print(F("AS3935")); break;
				    case 0x02: Serial.print(F("AS3935")); break;
				    case 0x03: Serial.print(F("AS3935")); break;
				    case 0x0A: Serial.print(F("SGTL5000")); break; // MCLK required
   				    case 0x0B: Serial.print(F("SMBusBattery?")); break;
				    case 0x0C: Serial.print(F("AK8963")); break;
				    case 0x10: Serial.print(F("CS4272")); break;
				    case 0x11: Serial.print(F("Si4713")); break;
				    case 0x13: Serial.print(F("VCNL4000,AK4558")); break;
  				    case 0x18: Serial.print(F("LIS331DLH,tempSensor")); break;
				    case 0x19: Serial.print(F("LSM303,LIS331DLH,tempSensor")); break;
				    case 0x1A: Serial.print(F("WM8731,tempSensor")); break;
				    case 0x1C: Serial.print(F("LIS3MDL,tempSensor")); break;
				    case 0x1D: Serial.print(F("LSM303D,LSM9DS0,ADXL345,MMA7455L,LSM9DS1,LIS3DSH,tempSensor")); break;
				    case 0x1E: Serial.print(F("LSM303D,HMC5883L,FXOS8700,LIS3DSH,tempSensor")); break;
                    case 0x1F: Serial.print(F("tempSensor")); break;
				    case 0x20: Serial.print(F("MCP23017,MCP23008,PCF8574,FXAS21002,SoilMoisture")); break;
				    case 0x21: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
				    case 0x22: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
				    case 0x23: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
				    case 0x24: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
				    case 0x25: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
				    case 0x26: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
				    case 0x27: Serial.print(F("MCP23017,MCP23008,PCF8574,LCD16x2,DigoleDisplay")); break;
				    case 0x28: Serial.print(F("BNO055,EM7180,CAP1188")); break;
				    case 0x29: Serial.print(F("TSL2561,VL6180,TSL2561,TSL2591,BNO055,CAP1188")); break;
				    case 0x2A: Serial.print(F("SGTL5000,CAP1188")); break;
				    case 0x2B: Serial.print(F("CAP1188")); break;
				    case 0x2C: Serial.print(F("MCP44XX ePot")); break;
				    case 0x2D: Serial.print(F("MCP44XX ePot")); break;
				    case 0x2E: Serial.print(F("MCP44XX ePot")); break;
				    case 0x2F: Serial.print(F("MCP44XX ePot")); break;
				    case 0x33: Serial.print(F("MAX11614,MAX11615")); break;
				    case 0x34: Serial.print(F("MAX11612,MAX11613")); break;
				    case 0x35: Serial.print(F("MAX11616,MAX11617")); break;
				    case 0x38: Serial.print(F("RA8875,FT6206,MAX98390")); break;
				    case 0x39: Serial.print(F("TSL2561, APDS9960")); break;
				    case 0x3C: Serial.print(F("SSD1306,DigisparkOLED")); break;
				    case 0x3D: Serial.print(F("SSD1306")); break;
  				    case 0x40: Serial.print(F("PCA9685,Si7021")); break;
				    case 0x41: Serial.print(F("STMPE610,PCA9685")); break;
				    case 0x42: Serial.print(F("PCA9685")); break;
				    case 0x43: Serial.print(F("PCA9685")); break;
				    case 0x44: Serial.print(F("PCA9685, SHT3X")); break;
				    case 0x45: Serial.print(F("PCA9685, SHT3X")); break;
				    case 0x46: Serial.print(F("PCA9685")); break;
				    case 0x47: Serial.print(F("PCA9685")); break;
				    case 0x48: Serial.print(F("ADS1115,PN532,TMP102,LM75,PCF8591")); break;
				    case 0x49: Serial.print(F("ADS1115,TSL2561,PCF8591")); break;
				    case 0x4A: Serial.print(F("ADS1115")); break;
				    case 0x4B: Serial.print(F("ADS1115,TMP102,BNO080")); break;
				    case 0x50: Serial.print(F("EEPROM,FRAM")); break;
				    case 0x51: Serial.print(F("EEPROM")); break;
				    case 0x52: Serial.print(F("Nunchuk,EEPROM")); break;
				    case 0x53: Serial.print(F("ADXL345,EEPROM")); break;
				    case 0x54: Serial.print(F("EEPROM")); break;
				    case 0x55: Serial.print(F("EEPROM")); break;
				    case 0x56: Serial.print(F("EEPROM")); break;
				    case 0x57: Serial.print(F("EEPROM")); break;
				    case 0x58: Serial.print(F("TPA2016,MAX21100")); break;
				    case 0x5A: Serial.print(F("MPR121")); break;
				    case 0x60: Serial.print(F("MPL3115,MCP4725,MCP4728,TEA5767,Si5351")); break;
				    case 0x61: Serial.print(F("MCP4725,AtlasEzoDO")); break;
				    case 0x62: Serial.print(F("LidarLite,MCP4725,AtlasEzoORP")); break;
				    case 0x63: Serial.print(F("MCP4725,AtlasEzoPH")); break;
				    case 0x64: Serial.print(F("AtlasEzoEC")); break;
				    case 0x66: Serial.print(F("AtlasEzoRTD")); break;
				    case 0x68: Serial.print(F("DS1307,DS3231,MPU6050,MPU9050,MPU9250,ITG3200,ITG3701,LSM9DS0,L3G4200D")); break;
				    case 0x69: Serial.print(F("MPU6050,MPU9050,MPU9250,ITG3701,L3G4200D")); break;
				    case 0x6A: Serial.print(F("LSM9DS1")); break;
				    case 0x6B: Serial.print(F("LSM9DS0")); break;
				    case 0x70: Serial.print(F("HT16K33")); break;
				    case 0x71: Serial.print(F("SFE7SEG,HT16K33")); break;
				    case 0x72: Serial.print(F("HT16K33")); break;
				    case 0x73: Serial.print(F("HT16K33")); break;
				    case 0x76: Serial.print(F("MS5607,MS5611,MS5637,BMP280,BME280,BME680")); break;
				    case 0x77: Serial.print(F("BMP085,BMA180,BMP280,BME280,BME680,MS5611")); break;
				    case 0x7C: Serial.print(F("FRAM_ID")); break;
				    default: Serial.print(F("unknown chip"));
			    }
	    }
#endif


/*!******************************************************************************************************
    @brief      Arduino method called once at startup to initialize the system
    @details    This is an Arduino IDE method which is called first upon boot or restart. It is only
                called one time and then control goes to the main "loop()" method, from which
                control never returns
    @return     void
********************************************************************************************************/
void setup()
{
    Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
    unsigned long debug_start = millis();
    while (!Serial && ((millis() - debug_start) <= 3000)) ;  // Wait for the GPS to startup and the Serial Interface.

    Serial.println("Project version: " + String(VERSION));  // Print Project version.
    Serial.println("Build timestamp:" + String(BUILD_TIMESTAMP));  // Print build timestamp

    delay(1000);                        // Time for the tempSensor sensor to perform an initial temperature conversion
    

	#if defined(findI2CAddresses)
		// uncomment these to use alternate pins
  		//Wire.setSCL(16);
  		//Wire.setSDA(17);
  		for (uint8_t wirelist_index = 0; wirelist_index < wirelist_count; wirelist_index++)
  			{
   	 			wireList[wirelist_index]._wire->begin();
  			}

  		Serial.println(F("\nI2C Scanner"));
	
	#else
    	Serial.print(F("Starting I2CDemo example program for BME680\n"));
    	Serial.print(F("- Initializing BME680 sensor\n"));
   	 	while (!BME680.begin(I2C_STANDARD_MODE))
        	{  // Start BME680 using I2C, use first device found
            	Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
            	delay(5000);
       	 	}  // End of loop until device is located.



    	Serial.print(F("- Setting 16x oversampling for all sensors\n"));
   	 	BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
    	BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
    	BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
		Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
		BME680.setIIRFilter(IIR4);  // Use enumerated type values
		Serial.print(F("- Setting gas measurement to 320\xC2\xB0 for 150ms\n"));  // "�C" symbols
		BME680.setGas(320, 150);  // 320°c for 150 milliseconds

        bool status = tempSensor.isConnected();  // initialize the hardware
        if ( status != 1 )
            {
                Serial << "Error, checking for connection status, status: " << status << endl;
                Serial.flush();
                while (1);                      // loop until reset
            }

 
        tempSensor.setTupper( (float)50 );           // 50C (122F)
        tempSensor.setTlower( (float)-10);          // -10C (14F)
        tempSensor.setTcritical( (float)60);        // 60C (140F)
    #endif
    
} // End of method setup()



/*!*****************************************************************************************************
    @brief      Arduino method for the main program loop
    @details    This is the main program for the Arduino IDE, it is an infinite loop and keeps on
                repeating. The "sprintf()" function is to pretty-print the values, since floating
                point is not supported on the Arduino, split the values into those before and those
                after the decimal point.
    @return   void
*******************************************************************************************************/
void loop()
{
	#if defined(findI2CAddresses)
		byte error, address;
  		int nDevices;

  		for (uint8_t wirelist_index = 0; wirelist_index < wirelist_count; wirelist_index++)
  			{
    			IMXRT_LPI2C_t * const port = wireList[wirelist_index].port;
    			Serial.print(F("Scanning("));
    			Serial.print(wireList[wirelist_index].wire_name);
   			 	Serial.println(F(")..."));
   				nDevices = 0;
    			for (address = 1; address < 127; address++)
    				{
     					 // The i2c_scanner uses the return value of
      					// the Write.endTransmisstion to see if
      					// a device did acknowledge to the address.
     					 wireList[wirelist_index]._wire->beginTransmission(address);
      					error = wireList[wirelist_index]._wire->endTransmission();

      					if (error == 0)
      						{
        						Serial.print(F("Device found at address 0x"));
        						if (address < 16)
        							{
          								Serial.print("0");
        							}
        						Serial.print(address, HEX);
        						Serial.print("  (");
       							printKnownChips(address);
        						Serial.println(")");

       							nDevices++;
      						}
      					else if (error == 4)
      						{
        						Serial.print(F("Unknown error at address 0x"));
        						if (address < 16)
        							{
          								Serial.print("0");
        							}
       							Serial.print(address, HEX);
        						Serial.printf("MCR:%x MSR:%x, MIER:%x MDER:%x MCFGR0:%x MDMR:%x MCCR0:%x\n",
          						port->MCR, port->MSR, port->MIER, port->MDER, port->MCFGR0, port->MDMR, port->MCCR0);
      						}
   					}
				if (nDevices == 0)
   				 	{
      					Serial.println(F("No I2C devices found\n"));
    				}
				else
					{
      					Serial.println(F("done\n"));
    				}
  			}
 		 delay(5000);           // wait 5 seconds for next scan
	#else


    	static int32_t  temp, humidity, pressure, gas;  // BME readings
		static char     buf[16];                        // sprintf text buffer
		static float    alt;                            // Temporary variable
		//static uint16_t loopCounter = 0;                // Display iterations
        // Print the heading for the BME680 data outputs.
		Serial.print(F("\n\n TempC Humid%   Press hPa    Alt m   Air MOhm"));
		Serial.print(F("\n ===== ======   =========    ======  ========\n"));

		BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings

        Serial.printf("%3d.%02d", (int8_t)(temp / 100), (uint8_t)(temp % 100));

	    Serial.printf("%3d.%03d", (int8_t)(humidity / 1000), (uint16_t)(humidity % 1000));  // Humidity milli-pct

		Serial.printf("%7d.%02d", (int16_t)(pressure / 100), (uint8_t)(pressure % 100));  // Pressure Pascals

		alt = altitude(pressure);  // temp altitude
		Serial.printf("    %5d.%02d", (int16_t)(alt), ((uint8_t)(alt * 100) % 100));  // Altitude meters

		Serial.printf("  %4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
            
        displayData();

		delay(10000);  // Wait 10s

	#endif
	            
}  // of method loop()
