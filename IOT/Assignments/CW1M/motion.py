import sys
import time
import Adafruit_DHT
import RPi.GPIO as GPIO
import smbus
import HD44780MCP
import MCP230XX

# DHT11 Sensor setup
DHT_SENSOR = 11
DHT_PIN = 4

# Light Sensor setup
if GPIO.RPI_REVISION == 1:
    bus = smbus.SMBus(0)
else:
    bus = smbus.SMBus(1)

class LightSensor():
    def __init__(self):
        self.DEVICE = 0x5c  # Default device I2C address
        self.ONE_TIME_HIGH_RES_MODE_1 = 0x20

    def convertToNumber(self, data):
        return ((data[1] + (256 * data[0])) / 1.2)

    def readLight(self):
        data = bus.read_i2c_block_data(self.DEVICE, self.ONE_TIME_HIGH_RES_MODE_1)
        return self.convertToNumber(data)

# LCD setup
i2cAddr = 0x21  # MCP23008/17 I2C address
MCP = MCP230XX.MCP230XX('MCP23008', i2cAddr)
blPin = 7  # Backlight pin
MCP.set_mode(blPin, 'output')
MCP.output(blPin, True)

LCD = HD44780MCP.HD44780(MCP, 1, -1, 2, [3, 4, 5, 6], rows=2, characters=16)

# Motion Sensor setup
motion_pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(motion_pin, GPIO.IN)

def main():
    sensor = LightSensor()
    try:
        while True:
            # Read DHT11 sensor
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
            if humidity is not None and temperature is not None:
                temp_hum_msg = 'Temp={0:0.1f}C Hum={1:0.1f}%'.format(temperature, humidity)
            else:
                temp_hum_msg = 'DHT11 Failed!'

            # Read light sensor
            light_level = sensor.readLight()
            light_msg = 'Light Level: {:.1f} lx'.format(light_level)

            # Read motion sensor
            if GPIO.input(motion_pin) == 1:
                motion_msg = 'Motion detected!'
            else:
                motion_msg = 'Nothing moves ...'

            # Display messages on LCD
            LCD.clear_display()
            LCD.display_string(temp_hum_msg)
            time.sleep(1)
            LCD.set_cursor(0, 1)
            LCD.display_string(light_msg)

            # Optionally display motion status on a new line or update as needed
            LCD.clear_display()  # Clear for motion message
            LCD.display_string(motion_msg)

            time.sleep(2)  # Adjust delay as needed

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
