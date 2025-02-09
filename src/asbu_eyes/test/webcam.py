import cv2

# Open the default webcam
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Set the webcam resolution to 1280x800
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break
    
    height, width = frame.shape[:2]

    # Display the resulting frame
    print(height, width)
    cv2.imshow('Webcam', frame)

    # Press 'q' on the keyboard to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close the window
cap.release()
cv2.destroyAllWindows()
