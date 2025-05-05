 import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Color mapping for fingers
finger_colors = {
    'thumb': (255, 0, 0),
    'index': (0, 255, 0),
    'middle': (0, 0, 255),
    'ring': (255, 255, 0),
    'pinky': (255, 0, 255)
}

# Helper function to calculate angles
def calculate_angle(a, b, c):
    a = np.array(a)  # Tip
    b = np.array(b)  # Middle joint
    c = np.array(c)  # Base joint
    
    ba = a - b
    bc = c - b
    
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))

    # Adjust to give 180° for open and 0° for closed
    return round(angle, 2)


# Connect to ESP32
ser = serial.Serial('COM6', 115200, timeout=1)
time.sleep(2)  # Allow ESP32 to reset


# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam. Check camera permissions!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            lm = hand_landmarks.landmark

            # Extract key points
            wrist = (lm[0].x, lm[0].y, lm[0].z)

            # Thumb joints
            thumb_cmc = (lm[1].x, lm[1].y, lm[1].z)
            thumb_mcp = (lm[2].x, lm[2].y, lm[2].z)
            thumb_ip = (lm[3].x, lm[3].y, lm[3].z)
            thumb_tip = (lm[4].x, lm[4].y, lm[4].z)

            # Index finger joints
            index_mcp = (lm[5].x, lm[5].y, lm[5].z)
            index_pip = (lm[6].x, lm[6].y, lm[6].z)
            index_dip = (lm[7].x, lm[7].y, lm[7].z)
            index_tip = (lm[8].x, lm[8].y, lm[8].z)

            # Middle finger joints
            middle_mcp = (lm[9].x, lm[9].y, lm[9].z)
            middle_pip = (lm[10].x, lm[10].y, lm[10].z)
            middle_dip = (lm[11].x, lm[11].y, lm[11].z)
            middle_tip = (lm[12].x, lm[12].y, lm[12].z)

            # Ring finger joints
            ring_mcp = (lm[13].x, lm[13].y, lm[13].z)
            ring_pip = (lm[14].x, lm[14].y, lm[14].z)
            ring_dip = (lm[15].x, lm[15].y, lm[15].z)
            ring_tip = (lm[16].x, lm[16].y, lm[16].z)

            # Pinky finger joints
            pinky_mcp = (lm[17].x, lm[17].y, lm[17].z)
            pinky_pip = (lm[18].x, lm[18].y, lm[18].z)
            pinky_dip = (lm[19].x, lm[19].y, lm[19].z)
            pinky_tip = (lm[20].x, lm[20].y, lm[20].z)

            # Calculate finger bending angles
            thumb_angle = 180 - calculate_angle(thumb_mcp, thumb_ip, thumb_tip)
            index_angle = 180 - calculate_angle(index_mcp, index_pip, index_tip)
            middle_angle = 180 - calculate_angle(middle_mcp, middle_pip, middle_tip)
            ring_angle = 180 - calculate_angle(ring_mcp, ring_pip, ring_tip)
            pinky_angle = 180 - calculate_angle(pinky_mcp, pinky_pip, pinky_tip)

            # Calculate thumb movement
            thumb_horizontal = calculate_angle(thumb_mcp, wrist, index_mcp)  # Sideways movement
            thumb_vertical = calculate_angle(thumb_cmc, thumb_mcp, thumb_tip)  # Up/down movement

            # Draw landmarks
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            

            # Display angles on screen
            cv2.putText(frame, f"Thumb Bend: {thumb_angle}°", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Thumb Horizontal: {thumb_horizontal}°", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(frame, f"Thumb Vertical: {thumb_vertical}°", (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            cv2.putText(frame, f"Index: {index_angle}°", (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Middle: {middle_angle}°", (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Ring: {ring_angle}°", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            cv2.putText(frame, f"Pinky: {pinky_angle}°", (50, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    # Show the webcam feed
    cv2.imshow("Hand Tracking - Finger Angles", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
