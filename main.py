"""Entry point for voice-guided robotic arm system."""

from command import command_handeler
from vision import yolo
from voice import VoiceRecognization


def main():
    """Listen for voice command, detect objects, and send to arm."""
    while True:
        print("\n--- Waiting for voice command ---")
        text = VoiceRecognization.listen_and_recognize()
        if not text:
            continue

        action, obj = VoiceRecognization.extract_command(text)
        if not action:
            print("No actionable command found.")
            continue

        print(f"Action: {action}, Object: {obj}")

        if obj:
            print("Running object detection...")
            detections = yolo.capture_and_detect()
            if detections:
                for d in detections:
                    print(f"  Detected: {d['class']} at {d['center']}")

        command_handeler.send_command(action)


if __name__ == "__main__":
    main()
