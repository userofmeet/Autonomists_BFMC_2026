import cv2
import numpy as np
import math
from collections import deque

# --- Global Buffers & State ---
lane_buffer = deque(maxlen=5)
angle_buffer = deque(maxlen=25) 
current_canny_low = 50
current_canny_high = 150

# --- Calibrated ROI Constants ---
LANE_ROI = [262, 480, 0, 640]
HAZARD_ROI = [117, 304, 72, 568]
SEMAPHORE_ROI = [174, 480, 320, 640]
SIGNAL_ROI = [0, 305, 321, 640]

def closed_loop_edge_detection(roi_frame):
    global current_canny_low, current_canny_high
    
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # Adaptive Thresholding for varying light
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)
    
    # Dynamic Canny Adjustment
    edges = cv2.Canny(thresh, current_canny_low, current_canny_high)
    edge_density = np.sum(edges == 255) / edges.size
    
    # Autocorrection Logic
    if edge_density > 0.03: 
        current_canny_low += 2
        current_canny_high += 2
    elif edge_density < 0.01:
        current_canny_low -= 2
        current_canny_high -= 2
        
    current_canny_low = np.clip(current_canny_low, 10, 100)
    current_canny_high = np.clip(current_canny_high, 100, 250)
    return edges

def average_slope_intercept(lines, y_offset):
    if lines is None: 
        return list(lane_buffer)[-1] if lane_buffer else []

    left_fit, right_fit = [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2: continue
            fit = np.polyfit((x1, x2), (y1 + y_offset, y2 + y_offset), 1)
            slope, intercept = fit[0], fit[1]
            if slope < -0.3: left_fit.append((slope, intercept))
            elif slope > 0.3: right_fit.append((slope, intercept))

    def make_pts(slope, intercept):
        y1, y2 = 480, 320
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return [[x1, y1, x2, y2]]

    lanes = []
    if left_fit: lanes.append(make_pts(*np.average(left_fit, axis=0)))
    if right_fit: lanes.append(make_pts(*np.average(right_fit, axis=0)))
    if lanes: lane_buffer.append(lanes)
    return lanes

def draw_hud(frame, lanes, steer_angle, x_offset):
    hud = frame.copy()
    
    # Draw ROIs
    cv2.rectangle(hud, (0, 262), (640, 480), (0, 255, 0), 2)       
    cv2.rectangle(hud, (72, 117), (568, 304), (0, 255, 255), 2)   
    cv2.rectangle(hud, (320, 174), (640, 480), (0, 0, 255), 2)    
    cv2.rectangle(hud, (321, 0), (640, 305), (255, 0, 0), 2)      

    # Dynamic Offset Line
    origin_x, origin_y = 320, 480
    target_x = 320 + x_offset
    cv2.line(hud, (origin_x, origin_y), (int(target_x), 300), (0, 0, 255), 4)

    # Lane Overlays
    for line in lanes:
        for x1, y1, x2, y2 in line:
            cv2.line(hud, (x1, y1), (x2, y2), (0, 255, 0), 5)
            
    cv2.putText(hud, f"STEER: {steer_angle-90:.1f}deg", (10, 40), 1, 1.5, (0,0,255), 2)
    return hud

# --- Main Execution ---
cap = cv2.VideoCapture(0)
while True:
    ret, img = cap.read()
    if not ret: break
    img = cv2.resize(img, (640, 480))
    
    # ROI Slicing
    lane_slice = img[LANE_ROI[0]:LANE_ROI[1], LANE_ROI[2]:LANE_ROI[3]]
    edges = closed_loop_edge_detection(lane_slice)
    
    # Detection
    line_segs = cv2.HoughLinesP(edges, 1, np.pi/180, 40, minLineLength=30, maxLineGap=100)
    lanes = average_slope_intercept(line_segs, LANE_ROI[0])
    
    # Math
    x_offset = 0
    if len(lanes) == 2:
        x_offset = (lanes[0][0][2] + lanes[1][0][2]) / 2 - 320
    elif len(lanes) == 1:
        x_offset = lanes[0][0][2] - lanes[0][0][0]

    raw_angle = math.degrees(math.atan(x_offset / 240)) + 90
    angle_buffer.append(raw_angle)
    smooth_angle = sum(angle_buffer) / len(angle_buffer)

    cv2.imshow("Autonomists - Final Integrated Engine", draw_hud(img, lanes, smooth_angle, x_offset))
    
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
