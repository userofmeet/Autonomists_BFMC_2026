import cv2
import numpy as np

def get_calibrated_rois(frame):
    # Hidden background processing (Selective Blindness) [cite: 13, 596]
    lane_slice = frame[262:480, 0:640]   # Lane Logic
    hazard_slice = frame[117:304, 72:568] # Pedestrian, Car, Obstacles
    sign_slice = frame[174:480, 320:640] # Static Sign Boards
    signal_slice = frame[0:305, 321:640] # Semaphores
    
    return lane_slice, hazard_slice, sign_slice, signal_slice

def draw_user_hud(frame):
    # Human-Machine Interface Layer [cite: 13, 216]
    hud = frame.copy()
    
    # 1. LANE ENGINE (Green)
    cv2.rectangle(hud, (0, 262), (640, 480), (0, 255, 0), 2)
    cv2.putText(hud, "LANE DETECTION", (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # 2. HAZARD ZONE: Cars/Pedestrians (Yellow) [cite: 98]
    cv2.rectangle(hud, (72, 117), (568, 304), (0, 255, 255), 2)
    cv2.putText(hud, "HAZARD ZONE (Cars/Pedestrians)", (80, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    # 3. SIGN TRACKER (Red) [cite: 95]
    cv2.rectangle(hud, (320, 174), (640, 480), (0, 0, 255), 2)
    cv2.putText(hud, "SEMAPHORE", (330, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # 4. TRAFFIC SIGNAL (Blue) [cite: 97]
    cv2.rectangle(hud, (321, 0), (640, 305), (255, 0, 0), 2)
    cv2.putText(hud, "TRAFFIC SIGNAL", (330, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    return hud

# Test logic
cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()
    if not ret: break
    img = cv2.resize(img, (640, 480))
    
    # Draw and Show
    display = draw_user_hud(img)
    cv2.imshow("Autonomists - HUD", display)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
