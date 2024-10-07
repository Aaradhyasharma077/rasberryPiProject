import cv2
import numpy as np

class ConveyorRelay:
    def __init__(self):
        self.is_active = False

    def turn_on(self):
        self.is_active = True
        print("Relay turned ON")
        GPIO.output(GPIO14,GPIO.LOW)
        # Add your actual relay on code here, e.g., GPIO.output(relay_pin, GPIO.HIGH)

    def turn_off(self):
        self.is_active = False
        print("Relay turned OFF")
        GPIO.output(GPIO14,GPIO.HIGH)
        # Add your actual relay off code here, e.g., GPIO.output(relay_pin, GPIO.LOW)

# Initialize the relay
relay = ConveyorRelay()

# Initialize video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()  # Updated to retrieve the return value
    if not ret:  # Check if the frame is successfully captured
        print("Failed to capture frame")
        break

        # Convert frame to grayscale for processing
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    belt = frame[209:327, 137:280]  # coordinates for the conveyor belt area
    gray_belt = cv2.cvtColor(belt, cv2.COLOR_BGR2GRAY)

    # Perform binary thresholding
    _, threshold = cv2.threshold(gray_belt, 50, 255, cv2.THRESH_BINARY)

    # Detect objects in the thresholded image
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:  # cnt represents the boundary of each detected object
        (x, y, w, h) = cv2.boundingRect(cnt)
        area = cv2.contourArea(cnt)

        # Distinguish small and big objects
        if area > 400:
            # Big nut detected
            cv2.rectangle(belt, (x, y), (x + w, y + h), (0, 0, 255), 2)
            relay.turn_off()  # Stop the belt if a big nut is detected
        elif 100 < area < 400:
            # Medium-sized object detected
            cv2.rectangle(belt, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Display the area of the detected object
        cv2.putText(belt, str(int(area)), (x, y), 1, 1, (0, 255, 0))

        # Display the frames
    cv2.imshow("Frame", frame)
    cv2.imshow("Belt", belt)
    cv2.imshow("Threshold", threshold)

    key = cv2.waitKey(1)
    if key == 27:  # ESC key to exit
        break
    elif key == ord("n"):  # Press 'n' to turn ON the relay
        relay.turn_on()
    elif key == ord("m"):  # Press 'm' to turn OFF the relay
        relay.turn_off()

cap.release()
cv2.destroyAllWindows()