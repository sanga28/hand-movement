import cv2
import mediapipe as mp
import numpy as np
import serial
import time

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

def calculate_angle(a, b, c):
    a, b, c = np.array(a), np.array(b), np.array(c)
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    return round(np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0))), 2)

def map_value(value, in_min=0, in_max=180, out_min=0, out_max=10):
    return round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 2)

# Connect to Arduino
ser = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("❌ Error: Could not read frame from webcam.")
        break

    # OPTIONAL: Flip the frame horizontally for mirror effect
    frame = cv2.flip(frame, 1)

    # Convert the frame to RGB for MediaPipe processing
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame to detect hands
    results = hands.process(frame_rgb)

    # OPTIONAL: Set a background color or overlay text before hand landmarks
    # frame[:] = (50, 50, 50)  # Uncomment to gray out the whole frame



    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            lm = hand_landmarks.landmark

            wrist = (lm[0].x, lm[0].y, lm[0].z)
            thumb_cmc = (lm[1].x, lm[1].y, lm[1].z)
            thumb_mcp = (lm[2].x, lm[2].y, lm[2].z)
            thumb_ip  = (lm[3].x, lm[3].y, lm[3].z)
            thumb_tip = (lm[4].x, lm[4].y, lm[4].z)

            index_mcp = (lm[5].x, lm[5].y, lm[5].z)
            index_pip = (lm[6].x, lm[6].y, lm[6].z)
            index_tip = (lm[8].x, lm[8].y, lm[8].z)

            middle_mcp = (lm[9].x, lm[9].y, lm[9].z)
            middle_pip = (lm[10].x, lm[10].y, lm[10].z)
            middle_tip = (lm[12].x, lm[12].y, lm[12].z)

            ring_mcp = (lm[13].x, lm[13].y, lm[13].z)
            ring_pip = (lm[14].x, lm[14].y, lm[14].z)
            ring_tip = (lm[16].x, lm[16].y, lm[16].z)

            pinky_mcp = (lm[17].x, lm[17].y, lm[17].z)
            pinky_pip = (lm[18].x, lm[18].y, lm[18].z)
            pinky_tip = (lm[20].x, lm[20].y, lm[20].z)

            def map_finger_angles():
                vertical_wrist_angle = calculate_angle(index_mcp, wrist, pinky_mcp)

                # Horizontal wrist estimation: difference in x-coordinates
                def calculate_wrist_horizontal():
                    dx = index_mcp[0] - pinky_mcp[0]  # how far index is from pinky on the x-axis
                    dx_mapped = np.interp(dx, [-0.2, 0.2], [0, 10])  # map to 0-10 range (adjust range if needed)
                    return round(np.clip(dx_mapped, 0, 10), 2)

                return {
                    "index_angle": map_value(180 - calculate_angle(index_mcp, index_pip, index_tip)),
                    "middle_angle": map_value(180 - calculate_angle(middle_mcp, middle_pip, middle_tip)),
                    "ring_angle": map_value(180 - calculate_angle(ring_mcp, ring_pip, ring_tip)),
                    "pinky_angle": map_value(180 - calculate_angle(pinky_mcp, pinky_pip, pinky_tip)),
                    "thumb_vertical": map_value(calculate_angle(thumb_cmc, thumb_mcp, thumb_tip)),
                    "thumb_horizontal": map_value(calculate_angle(thumb_cmc, thumb_mcp, thumb_tip)),
                    "wrist_angle": map_value(vertical_wrist_angle),
                    "wrist_horizontal": calculate_wrist_horizontal()
                    }


            mapped_values = map_finger_angles()

            data =  f"ind_{int(mapped_values['index_angle'])}:" \
                    f"mid_{int(mapped_values['middle_angle'])}:" \
                    f"rin_{int(mapped_values['ring_angle'])}:" \
                    f"lit_{int(mapped_values['pinky_angle'])}:" \
                    f"thu_{int(mapped_values['thumb_vertical'])}:" \
                    f"ths_{int(mapped_values['thumb_horizontal'])}:" \
                    f"wri_{int(mapped_values['wrist_angle'])}:" \
                    f"wrh_{int(mapped_values['wrist_horizontal'])}\n"


            ser.write(data.encode())

            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            y = 50
            for k, v in mapped_values.items():
                cv2.putText(frame, f"{k}: {v}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                y += 25

    cv2.imshow("Hand Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()