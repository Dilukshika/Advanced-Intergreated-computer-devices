# Smart Home Monitoring Energy Efficiency System (Sample Crow Pi code)
# Student Name: Dilukshika
# Student ID: 30073395

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

# Buzzer, Button, LED and Motion setup
button_pin = 26
buzzer_pin = 18
led_pin = 19
motion_pin = 23

# set board mode to GPIO.BOARD
GPIO.setmode(GPIO.BCM)

# setup button pin asBu input and buzzer pin as output and puin as input
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buzzer_pin, GPIO.OUT)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(motion_pin, GPIO.IN)


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

# State to track whether the system is on or off
system_on = False

def toggle_system(channel):
    global system_on
    system_on = not system_on  # Toggle system state

# Button interrupt setup
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=toggle_system, bouncetime=300)

def main():
    sensor = LightSensor()
    try:
        while True:
            if system_on:
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
                time.sleep(1)
                LCD.clear_display()
                LCD.display_string(motion_msg)
                
                # Buzzer and LED alarm conditions
                if light_level < 2 or (temperature is not None and temperature < 10):
                    # Turn on both buzzer and LED
                    GPIO.output(buzzer_pin, GPIO.HIGH)
                    GPIO.output(led_pin, GPIO.HIGH)
                    time.sleep(3)
                    # Turn off both buzzer and LED
                    GPIO.output(buzzer_pin, GPIO.LOW)
                    GPIO.output(led_pin, GPIO.LOW)

                time.sleep(2)  # Delay between readings
            else:
                LCD.clear_display()  # Clear LCD if system is off

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
