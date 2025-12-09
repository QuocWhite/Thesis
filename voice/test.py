import speech_recognition as sr

# Define possible command keywords
COMMAND_KEYWORDS = ['take', 'pick', 'grab', 'get', 'put']


def extract_command(text):
    for keyword in COMMAND_KEYWORDS:
        if keyword in text.lower():
            return keyword
    return None


# Initialize recognizer
recognizer = sr.Recognizer()

with sr.Microphone() as source:
    print("Say your command:")
    audio = recognizer.listen(source)

try:
    # Use Google Speech Recognition (online) or
    # change to use offline engines if needed
    command_text = recognizer.recognize_google(audio)
    print("You said:", command_text)
    command = extract_command(command_text)
    if command:
        print("Extracted Command:", command)
    else:
        print("No known command found.")
except Exception as e:
    print("Error recognizing speech:", e)
