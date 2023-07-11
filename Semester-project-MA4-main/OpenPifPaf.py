import cv2
import openpifpaf

# Load the OpenPifPaf model for keypoint detection
model, _ = openpifpaf.network.factory(checkpoint='shufflenetv2k16')

# Create a VideoCapture object to capture frames from the camera
cap = cv2.VideoCapture(0)

# Create an instance of the OpenPifPaf keypoint tracker
tracker = openpifpaf.tracker.Tracker()

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to RGB color space (OpenPifPaf requires RGB input)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect and track keypoints in the frame using OpenPifPaf
    with openpifpaf.datasets.penn.PennVideo.process_frame(model, frame, visualize=False) as predictions:
        tracker.predict(predictions)
        tracker.update(predictions)

    # Draw the keypoints on the frame
    frame = openpifpaf.show.KeypointPainter.show(
        frame, predictions, show_box=False, color_connections=True)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close the window
cap.release()
cv2.destroyAllWindows()
