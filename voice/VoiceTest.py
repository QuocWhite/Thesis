import speech_recognition as sr

mic_index = 2  # <-- replace with your Bluetooth mic's index from above

recognizer = sr.Recognizer()
mic = sr.Microphone(device_index=mic_index)

with mic as source:
    print("Say something...")
    recognizer.adjust_for_ambient_noise(source)
    audio = recognizer.listen(source)
    print("Recording finished.")

with open("test_bluetooth.wav", "wb") as f:
    f.write(audio.get_wav_data())
print("Saved as test_bluetooth.wav.")
