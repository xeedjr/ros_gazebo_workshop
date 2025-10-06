import cv2

def calculate_focus_measure(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return lap_var

def focus_assist(camera_index=2):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3200)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return

    print("🎥 Press ESC to exit")
    while True:
        ret, frame_inp = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        frame = frame_inp[:, :frame_inp.shape[1] // 2]
        #frame = frame_inp[:, frame_inp.shape[1] // 2:]

        focus_score = calculate_focus_measure(frame)
        display_frame = frame.copy()
        cv2.putText(display_frame, f"Focus Score: {focus_score:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Focus Assist", display_frame)

        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    focus_assist()
