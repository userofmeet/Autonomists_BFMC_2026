import cv2
import numpy as np

def nothing(x):
    pass

def main():
    cap = cv2.VideoCapture(0)
    cv2.namedWindow("Master ROI Controls")
    cv2.resizeWindow("Master ROI Controls", 800, 600)

    # Helper to create 4 sliders for a specific ROI
    def create_roi_sliders(name, default_vals):
        cv2.createTrackbar(f"{name} X1", "Master ROI Controls", default_vals[0], 640, nothing)
        cv2.createTrackbar(f"{name} X2", "Master ROI Controls", default_vals[1], 640, nothing)
        cv2.createTrackbar(f"{name} Y1", "Master ROI Controls", default_vals[2], 480, nothing)
        cv2.createTrackbar(f"{name} Y2", "Master ROI Controls", default_vals[3], 480, nothing)

    # Initial Defaults [X1, X2, Y1, Y2]
    create_roi_sliders("Lane", [0, 640, 350, 480])
    create_roi_sliders("Sign", [450, 640, 20, 200])
    create_roi_sliders("Signal", [320, 450, 20, 200]) # The missing Traffic Signal ROI
    create_roi_sliders("Obs", [200, 440, 250, 400])

    print("JARVIS: Full Manual Override active. Adjust all axes as needed.")

    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.resize(frame, (640, 480))
        hud = frame.copy()

        def get_coords(name):
            x1 = cv2.getTrackbarPos(f"{name} X1", "Master ROI Controls")
            x2 = cv2.getTrackbarPos(f"{name} X2", "Master ROI Controls")
            y1 = cv2.getTrackbarPos(f"{name} Y1", "Master ROI Controls")
            y2 = cv2.getTrackbarPos(f"{name} Y2", "Master ROI Controls")
            return (x1, y1), (x2, y2)

        # 1. Lane (Green)
        l1, l2 = get_coords("Lane")
        cv2.rectangle(hud, l1, l2, (0, 255, 0), 2)
        
        # 2. Signs (Red)
        s1, s2 = get_coords("Sign")
        cv2.rectangle(hud, s1, s2, (0, 0, 255), 2)
        
        # 3. Traffic Signal (Blue)
        sig1, sig2 = get_coords("Signal")
        cv2.rectangle(hud, sig1, sig2, (255, 0, 0), 2)
        
        # 4. Obstacles (Yellow)
        o1, o2 = get_coords("Obs")
        cv2.rectangle(hud, o1, o2, (0, 255, 255), 2)

        cv2.imshow("Autonomists - Total Control HUD", hud)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("\n--- SAVED COORDINATES ---")
            for n in ["Lane", "Sign", "Signal", "Obs"]:
                coords = get_coords(n)
                print(f"{n} ROI: {coords}")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
