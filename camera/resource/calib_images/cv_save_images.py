import os
import cv2

# Create a directory to save images if it doesn't exist
save_dir = "checkerboard"
os.makedirs(save_dir, exist_ok=True)

# Initialize the video capture object
cap = cv2.VideoCapture(6)

# Check if the webcam is opened successfully
if not cap.isOpened():
    print("Error: Unable to access webcam")
    exit()

# Create a window to display the video stream
cv2.namedWindow("Video Stream")

# Variable to keep track of the image count
img_count = 0

# Infinite loop to continuously stream video
while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # Check if the frame is successfully captured
    if not ret:
        print("Error: Unable to capture frame")
        break

    # Display the frame
    cv2.imshow("Video Stream", frame)

    # Wait for a key press
    key = cv2.waitKey(1)

    # Check if spacebar is pressed (keycode for spacebar is 32)
    if key == ord('s'):
        # Save the current frame as an image
        img_count += 1
        img_name = os.path.join(save_dir, "image_" + str(img_count) + ".jpg")
        cv2.imwrite(img_name, frame)
        print("Image saved as", img_name)

    # Check if the 'q' key is pressed to exit the loop
    elif key == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
