import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import threading
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


class Semaphore:
    def __init__(self, topic, led_pins, sonar_pins, motion_pin, buzzer_pin):
        self.topic = topic
        self.led_pins = led_pins
        self.sonar_pins = sonar_pins
        self.motion_pin = motion_pin
        self.buzzer_pin = buzzer_pin
        for pin in led_pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(sonar_pins[1], GPIO.IN)
        GPIO.setup(sonar_pins[0], GPIO.OUT)
        GPIO.setup(motion_pin, GPIO.IN)
        GPIO.setup(buzzer_pin, GPIO.OUT, initial=GPIO.LOW)

    def get(self):
        time.sleep(0.5)
        GPIO.output(self.sonar_pins[0], GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.sonar_pins[0], GPIO.LOW)
        pulse_start_time = 0
        while GPIO.input(self.sonar_pins[1]) == 0:
            pulse_start_time = time.time()
        pulse_end_time = 0
        while GPIO.input(self.sonar_pins[1]) == 1:
            pulse_end_time = time.time()
        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)
        return (distance, GPIO.input(self.motion_pin))

    def set(self, value):
        if value == 'G':
            GPIO.output(self.buzzer_pin, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(self.buzzer_pin, GPIO.LOW)
        GPIO.output(self.led_pins[0], GPIO.HIGH if value == 'G' else GPIO.LOW)
        GPIO.output(self.led_pins[1], GPIO.HIGH if value == 'Y' else GPIO.LOW)
        GPIO.output(self.led_pins[2], GPIO.HIGH if value == 'R' else GPIO.LOW)


SEMAPHORES = [
    Semaphore('wot-semaphore/fst', [8, 10, 12], [7, 11], 19, 3),
    Semaphore('wot-semaphore/snd', [16, 18, 22], [13, 15], 21, 5),
]


def publish(client):
    while True:
        for semaphore in SEMAPHORES:
            distance, motion = semaphore.get()
            client.publish(semaphore.topic + '/sonar', payload=distance)
            print('Distance:', distance, 'cm')
            client.publish(semaphore.topic + '/motion', payload=motion)
            print('Motion:', motion)


def on_connect(client, userdata, flags, rc):
    print('Connected with result code ' + str(rc))
    for semaphore in SEMAPHORES:
        client.subscribe(semaphore.topic + '/led')
    threading.Thread(target=publish, args=(client,)).start()


def on_message(client, userdata, msg):
    print(msg.topic + ' ' + str(msg.payload))
    for semaphore in SEMAPHORES:
        if msg.topic == semaphore.topic + '/led':
            semaphore.set(msg.payload)
            break


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('broker.mqttdashboard.com', 1883, 60)
    client.loop_forever()


if __name__ == '__main__':
    main()

GPIO.cleanup()
