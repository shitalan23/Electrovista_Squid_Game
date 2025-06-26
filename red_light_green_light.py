import cv2
import numpy as np
import serial
import time

# Initialize camera feed
cap = cv2.VideoCapture(0)

# Define color ranges for green and red clothing
LOWER_RED = np.array([0, 100, 100])
UPPER_RED = np.array([10, 255, 255])

# Fine-tuned HSV range for green
LOWER_GREEN = np.array([35, 50, 50])  # Lower bound for green
UPPER_GREEN = np.array([85, 255, 255])  # Upper bound for green

GUARD_BOXES = [(100, 100, 200, 300), (400, 50, 150, 250)]
MIN_CONTOUR_AREA = 500
fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16)
red_light_active, homography_matrix, calibrated = True, None, False

# Initialize serial connection with Arduino
arduino_port = "/dev/ttyACM0"  # Replace with your Arduino's port
baud_rate = 9600
try:
    arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Wait for Arduino to initialize
    print("Connected to Arduino!")
except Exception as e:
    print(f"Error connecting to Arduino: {e}")
    exit()

def calibrate_homography():
    global homography_matrix, calibrated
    real_points = np.array([[0, 0], [5, 0], [5, 5], [0, 5]], dtype=np.float32)
    pixel_points, frame = [], None

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pixel_points.append((x, y))
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow('Red Light Green Light', frame)

    while len(pixel_points) < 4:
        ret, frame = cap.read()
        if not ret: break
        cv2.putText(frame, "Click on 4 corners of the play area", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Red Light Green Light', frame)
        cv2.setMouseCallback('Red Light Green Light', mouse_callback)
        if cv2.waitKey(1) == ord('q'): break

    if len(pixel_points) == 4:
        homography_matrix, _ = cv2.findHomography(np.array(pixel_points, dtype=np.float32), real_points)
        calibrated = True
        print("Homography Matrix Calibrated!")
    else: print("Calibration failed. Please try again.")

def map_to_servo(x, y, frame_width, frame_height):
    """Map pixel coordinates to servo angles."""
    pan_angle = int(np.interp(x, [0, frame_width], [0, 180]))  # Map X to pan angle (0-180)
    tilt_angle = int(np.interp(y, [0, frame_height], [150, 30]))  # Map Y to tilt angle (30-150)
    return pan_angle, tilt_angle

while True:
    ret, frame = cap.read()
    if not ret: break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create green mask with fine-tuned HSV range
    green_mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    # Enhance green mask with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)  # Remove noise
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)  # Fill gaps
    green_mask = cv2.GaussianBlur(green_mask, (5, 5), 0)  # Smooth the mask

    # Red mask (unchanged)
    red_mask = cv2.inRange(hsv, LOWER_RED, UPPER_RED)

    # Motion mask (unchanged)
    motion_mask = fgbg.apply(frame) if red_light_active else None
    combined_mask = cv2.bitwise_and(green_mask, motion_mask) if red_light_active else green_mask

    # Find contours in the green mask
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_players, eliminated_players = [], []

    for cnt in contours:
        # Filter contours by area
        if cv2.contourArea(cnt) < MIN_CONTOUR_AREA: continue

        # Get bounding box
        x, y, w, h = cv2.boundingRect(cnt)

        # Skip if the contour overlaps with guard boxes
        if any((x < gx + gw and x + w > gx and y < gy + gh and y + h > gy) for (gx, gy, gw, gh) in GUARD_BOXES): continue

        # Create a mask for the contour
        contour_mask = np.zeros_like(red_mask)
        cv2.drawContours(contour_mask, [cnt], -1, 255, -1)

        # Skip if the contour contains significant red pixels
        if cv2.countNonZero(cv2.bitwise_and(contour_mask, red_mask)) > 0.1 * cv2.contourArea(cnt): continue

        # Add valid player
        valid_players.append((x, y, w, h))

    for (x, y, w, h) in valid_players:
        centroid = (x + w // 2, y + h // 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, centroid, 5, (0, 0, 255), -1)

        if calibrated:
            real_coords = cv2.perspectiveTransform(np.array([[centroid]], dtype=np.float32), homography_matrix)[0][0]
            print(f"Player at pixel {centroid} -> Real-world ({real_coords[0]:.2f}, {real_coords[1]:.2f})")

        if red_light_active and cv2.countNonZero(motion_mask[y:y+h, x:x+w]) > 0.1 * (w * h):
            eliminated_players.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, "ELIMINATED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Aim laser at the player
            pan_angle, tilt_angle = map_to_servo(centroid[0], centroid[1], frame.shape[1], frame.shape[0])
            command = f"{pan_angle},{tilt_angle}\n"
            arduino.write(command.encode())
            print(f"Sent command to Arduino: {command.strip()}")

    phase_text = "RED LIGHT - NO MOVEMENT!" if red_light_active else "GREEN LIGHT - MOVE!"
    cv2.putText(frame, phase_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255) if red_light_active else (0, 255, 0), 2)
    cv2.imshow('Red Light Green Light', frame)

    key = cv2.waitKey(1)
    if key == ord(' '): red_light_active = not red_light_active
    elif key == ord('q'): break
    elif key == ord('c') and not calibrated: calibrate_homography()

cap.release()
cv2.destroyAllWindows()
arduino.close()

