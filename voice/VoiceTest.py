import speech_recognition as sr


def extract_command(text):
    # Simple parsing for demonstration: looks for 'take the' and object color/type
    if 'take the' in text:
        words = text.split()
        try:
            idx = words.index('the')
            object_description = ' '.join(words[idx+1:])
            command = f"command take object {object_description}"
            return command
        except ValueError:
            return "command not recognized"
    else:
        return "command not recognized"


# Initialize recognizer
recognizer = sr.Recognizer()

# Use microphone input
with sr.Microphone() as source:
    print("Commander: Please say your command...")
    audio = recognizer.listen(source)

    try:
        # Speech-to-Text (STT)
        text_command = recognizer.recognize_google(audio)
        print("You said:", text_command)
        # Extract command
        extracted = extract_command(text_command.lower())
        print(extracted)
    except sr.UnknownValueError:
        print("Sorry, could not understand audio")
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))
