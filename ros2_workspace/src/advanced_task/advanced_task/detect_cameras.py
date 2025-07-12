import cv2
import sys
import os

def get_camera_name(index):
    """
    Returns the camera model name for a given video index by reading
    /sys/class/video4linux/videoX/name on Linux systems.
    If not found, returns 'Unknown'.
    """
    sys_path = f"/sys/class/video4linux/video{index}/name"
    try:
        with open(sys_path, 'r') as f:
            return f.read().strip()
    except FileNotFoundError:
        return "Unknown"

def find_available_cameras_with_names(max_index=5):
    """
    Probes video indices from 0 to max_index - 1.
    For each available camera:
      - Suppresses OpenCV stderr warnings while opening
      - Checks if the device can be opened
      - Reads and returns the camera name
    Returns a list of (index, name) tuples.
    """
    available = []

    for i in range(max_index):
        # Temporarily suppress stderr to hide OpenCV warnings
        stderr_fileno = sys.stderr.fileno()
        saved_stderr = os.dup(stderr_fileno)
        with open(os.devnull, 'w') as devnull:
            os.dup2(devnull.fileno(), stderr_fileno)
            try:
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    name = get_camera_name(i)
                    available.append((i, name))
                    cap.release()
            except Exception:
                pass
            finally:
                os.dup2(saved_stderr, stderr_fileno)
                os.close(saved_stderr)

    return available

if __name__ == "__main__":
    cameras = find_available_cameras_with_names(5)
    if cameras:
        print("Available cameras:")
        for idx, name in cameras:
            print(f"  Index {idx}: {name}")
    else:
        print("No cameras found!")
