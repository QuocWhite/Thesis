import speech_recognition as sr
import noisereduce as nr
import numpy as np


def listen_and_recognize():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        print("Calibrating for ambient noise... Please be quiet.")
        recognizer.adjust_for_ambient_noise(source, duration=2)
        print("Please speak your command...")
        audio = recognizer.listen(source)
        # Convert audio to numpy array for processing
        audio_data = np.frombuffer(audio.get_raw_data(), np.int16)
        # Reduce noise
        reduced_noise = nr.reduce_noise(y=audio_data, sr=16000)
        # Convert back to AudioData for recognition
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
    text = text.lower()
    actions = ['take', 'pick', 'grab', 'get']
    action = None
    for act in actions:
        if act in text:
            action = act
            break
    if action:
        # Remove polite words and the action
        for prefix in ['please', 'could you', 'would you']:
            text = text.replace(prefix, '')
        # Remove action
        text = text.replace(action, '', 1)
        object_ = text.strip()
        print(f'action: {action}')
        print(f'object: {object_}')
        return action, object_
    else:
        print("No action found in the command.")
        return None, None


if __name__ == '__main__':
    text = listen_and_recognize()
    if text:
        extract_command(text)
