import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import ssl
import json
import _thread
from mfrc522 import SimpleMFRC522

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# LED and Servo Setup
GREEN_LED = 13
RED_LED = 19
SERVO_PIN = 18

GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM for servo
servo.start(0)

# Keypad setup
inputs = [12, 26, 24, 23]
outputs = [21, 20, 16]
GPIO.setup(inputs, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(outputs, GPIO.OUT)

r1 = ['1', '2', '3']
r2 = ['4', '5', '6']
r3 = ['7', '8', '9']
r4 = ['*', '0', '#']

correct_passcode_unlock = ['1', '2', '3', '4']
correct_passcode_lock = ['4', '3', '2', '1']

entered_passcode = []

# RFID setup
reader = SimpleMFRC522()

def on_connect(client, userdata, flags, rc): 
    print("Connected with result code " + str(rc))

client = mqtt.Client()
client.on_connect = on_connect
client.tls_set(ca_certs="./rootCA.pem", certfile="Certificate.pem.crt", keyfile="./private.pem.key", tls_version=ssl.PROTOCOL_SSLv23)
client.tls_insecure_set(True)
client.connect("a19xi9eqyb00zu-ats.iot.us-east-1.amazonaws.com", 8883, 60)

def check_keypad():
    global entered_passcode
    while True:
        for pin, col in zip(outputs, range(3)):
            GPIO.output(pin, GPIO.HIGH)

            row = None
            if GPIO.input(12): row = r1
            elif GPIO.input(26): row = r2
            elif GPIO.input(24): row = r3
            elif GPIO.input(23): row = r4

            if row:
                key = row[col]
                print(f"[KEYPAD] Pressed: {key}")
                entered_passcode.append(key)
                time.sleep(0.3)

                # Check passcode
                if entered_passcode == correct_passcode_unlock:
                    door_unlocked()
                    reset_passcode()
                elif entered_passcode == correct_passcode_lock:
                    door_locked()
                    reset_passcode()
                elif len(entered_passcode) >= 4:
                    print("[KEYPAD] Incorrect passcode!")
                    flash_red_led()
                    reset_passcode()

            GPIO.output(pin, GPIO.LOW)
        time.sleep(0.05)  # Debounce delay

def check_rfid():
    last_correct_scan_time = 0
    correct_scan_count = 0

    while True:
        id, text = reader.read()
        text = text.strip()
        current_time = time.time()

        # Auto-reset after 10 seconds if no second scan
        if correct_scan_count > 0 and current_time - last_correct_scan_time > 10:
            print("[RFID] Timeout - Counter reset.")
            correct_scan_count = 0

        if text == "card":
            if current_time - last_correct_scan_time <= 10:
                correct_scan_count += 1
            else:
                correct_scan_count = 1  # Treat as first scan again

            last_correct_scan_time = current_time

            if correct_scan_count == 1:
                lock = "Access Granted - Unlocked"
                print(lock)
                door_unlocked()
            elif correct_scan_count == 2:
                lock = "Access Granted - Locked"
                print(lock)
                door_locked()
                correct_scan_count = 0  # Reset after locking

        elif text == "tag":
            lock = "No Access Granted"
            print(lock)
            flash_red_led()
            correct_scan_count = 0  # Reset on invalid scan

        client.publish("Rfid/data", payload=json.dumps({"lock": lock}), qos=0, retain=False)
        time.sleep(0.3)

def door_unlocked():
    print("Door unlocked!")
    flash_green_led()
    move_servo(90)
    client.publish("Rfid/data", payload=json.dumps({"lock": "Door unlocked"}), qos=0, retain=False)
    time.sleep(1)

def door_locked():
    print("Door locked!")
    move_servo(0)
    client.publish("Rfid/data", payload=json.dumps({"lock": "Door locked"}), qos=0, retain=False)

def flash_red_led():
    for _ in range(3):
        GPIO.output(RED_LED, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(RED_LED, GPIO.LOW)
        time.sleep(0.2)

def flash_green_led():
    for _ in range(3):
        GPIO.output(GREEN_LED, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(GREEN_LED, GPIO.LOW)
        time.sleep(0.2)

def move_servo(angle):
    duty = angle / 18 + 2  # Convert angle to duty cycle
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)  # Prevent servo jitter

def reset_passcode():
    global entered_passcode
    entered_passcode = []

# Run both RFID and Keypad in separate threads
try:
    # Start the keypad thread
    _thread.start_new_thread(check_keypad, ())

    # Start the RFID thread
    _thread.start_new_thread(check_rfid, ())

    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
finally:
    servo.stop()
    GPIO.cleanup()