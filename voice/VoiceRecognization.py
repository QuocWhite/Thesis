"""Speech-to-text module using Google Speech Recognition with noise reduction."""

import speech_recognition as sr
import noisereduce as nr
import numpy as np


def listen_and_recognize():
    """Capture microphone input and convert to text."""
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        print("Calibrating for ambient noise... Please be quiet.")
        recognizer.adjust_for_ambient_noise(source, duration=2)
        print("Please speak your command...")
        audio = recognizer.listen(source)
        audio_data = np.frombuffer(audio.get_raw_data(), np.int16)
        reduced_noise = nr.reduce_noise(y=audio_data, sr=16000)
        audio = sr.AudioData(reduced_noise.tobytes(),
                             audio.sample_rate, audio.sample_width)
    try:
        text = recognizer.recognize_google(audio)
        print("You said:", text)
        return text
    except sr.UnknownValueError:
        print("Could not understand audio.")
        return None
    except sr.RequestError:
        print("Request to Google Speech Recognition failed.")
        return None


def extract_command(text):
    """Parse action and object from a voice text command."""
    text = text.lower()
    actions = ["take", "pick", "grab", "get"]
    action = None
    for act in actions:
        if act in text:
            action = act
            break
    if action:
        for prefix in ["please", "could you", "would you"]:
            text = text.replace(prefix, "")
        text = text.replace(action, "", 1)
        object_ = text.strip()
        print(f"action: {action}")
        print(f"object: {object_}")
        return action, object_
    else:
        print("No action found in the command.")
        return None, None


if __name__ == "__main__":
    text = listen_and_recognize()
    if text:
        extract_command(text)
