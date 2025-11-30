---
id: voice-to-action
title: Voice-to-Action Pipeline with OpenAI Whisper
sidebar_position: 2
---

## 1. Introduction to Voice-to-Action in Robotics

In the realm of Vision-Language-Action (VLA) robotics, enabling robots to understand and respond to human voice commands is a pivotal capability. The **Voice-to-Action pipeline** describes the sequence of steps that transforms spoken language into executable robot behaviors. This pipeline typically involves speech recognition, natural language understanding (NLU), and the translation of understood commands into robot-specific actions.

The goal is to create a natural and intuitive interface where humans can communicate with robots using their voice, bypassing the need for complex programming or graphical user interfaces for common tasks.

## 2. OpenAI Whisper Integration for Speech Recognition

**OpenAI Whisper** is a general-purpose, open-source automatic speech recognition (ASR) system. It has been trained on a massive dataset of diverse audio and performs exceptionally well across various languages, accents, and noisy environments. Its high accuracy and robust performance make it an excellent choice for the speech recognition component of a voice-to-action pipeline in robotics.

### Speech Recognition Setup

To use OpenAI Whisper, you typically interact with it via the OpenAI API (for cloud-based transcription) or by running a local model (if you have sufficient computational resources and prefer offline processing). For robotic applications, the API is often preferred for its ease of use and high accuracy, provided internet connectivity is stable.

**Installation (OpenAI Python Library):**
First, ensure you have the OpenAI Python library installed:
```bash
pip install openai
```

**API Key Configuration:**
As emphasized previously, manage your OpenAI API key securely. The recommended method for development is via an environment variable.
```bash
# Set in your terminal (Linux/macOS)
export OPENAI_API_KEY="YOUR_API_KEY_HERE"

# For Windows PowerShell
# $env:OPENAI_API_KEY="YOUR_API_KEY_HERE"
```

### Capturing Audio

Before Whisper can transcribe, the robot needs to capture audio. This typically involves:
*   **Microphone Hardware**: A USB microphone or an array of microphones integrated into the robot.
*   **Audio Capture Software**: Libraries like `pyaudio` in Python can interface with microphones to record audio streams. In ROS 2, dedicated audio packages (e.g., `ros2_audio_driver`) might be used to publish audio data to ROS 2 topics.

## 3. Voice Command Processing with Whisper API

Once audio is captured, it needs to be processed by Whisper. The Whisper API expects an audio file.

**Conceptual Python Code Example using Whisper API:**

```python
import openai
import os
import io
import wave # For handling WAV files (common format)

# --- Configuration ---
# Ensure OPENAI_API_KEY is set as an environment variable
openai.api_key = os.getenv("OPENAI_API_KEY")

# --- Function to simulate audio capture (replace with actual microphone input) ---
def simulate_audio_capture(text_to_say="Hello robot, navigate to the kitchen."):
    """
    This function would typically record audio from a microphone.
    For demonstration, we'll create a dummy WAV file.
    In a real robot, you'd record actual speech.
    """
    # This is a placeholder. A real implementation would use pyaudio or a ROS audio driver
    # to record from the microphone and save to a temporary WAV file.
    dummy_wav_path = "/tmp/recorded_command.wav"
    with wave.open(dummy_wav_path, 'wb') as wf:
        wf.setnchannels(1) # mono
        wf.setsampwidth(2) # 16-bit
        wf.setframerate(16000) # 16kHz
        # Write dummy audio data (e.g., silence or synthetic speech)
        wf.writeframes(b'\x00\x00' * 16000) # 1 second of "silence"
    print(f"Simulated audio recorded to {dummy_wav_path}")
    return dummy_wav_path

# --- Function to transcribe audio using OpenAI Whisper API ---
def transcribe_audio_with_whisper(audio_file_path):
    try:
        with open(audio_file_path, "rb") as audio_file:
            print(f"Sending {audio_file_path} to Whisper API...")
            # For older OpenAI library versions:
            # transcript = openai.Audio.transcribe("whisper-1", audio_file)
            # For newer OpenAI library versions (recommended):
            transcript = openai.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file
            )
        return transcript.text
    except openai.APIConnectionError as e:
        print(f"OpenAI API Connection Error: {e}")
        print("Please check your internet connection and API key.")
        return None
    except openai.APIStatusError as e:
        print(f"OpenAI API Status Error: {e.status_code} - {e.response}")
        print("Please check your API key and request parameters.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

# --- Main voice-to-action processing function ---
def process_voice_command():
    print("Robot is listening for your command...")
    audio_path = simulate_audio_capture() # Simulate recording

    if audio_path:
        transcribed_text = transcribe_audio_with_whisper(audio_path)
        if transcribed_text:
            print(f"Transcribed Text: '{transcribed_text}'")
            # --- Next Step: Natural Language Understanding (NLU) ---
            # This is where an LLM (e.g., GPT) would take the transcribed text
            # and extract intent, entities, and parameters for robot actions.
            print(f"Sending '{transcribed_text}' for NLU and action planning...")
            return transcribed_text
    return None

if __name__ == "__main__":
    if not os.getenv("OPENAI_API_KEY"):
        print("Error: OPENAI_API_KEY environment variable not set.")
        print("Please set it before running: export OPENAI_API_KEY='YOUR_KEY'")
    else:
        processed_command = process_voice_command()
        if processed_command:
            print("Voice command pipeline completed for transcription stage.")
```

## 4. Converting Voice Commands to Robot Actions

Once Whisper has transcribed the spoken command into text, the next critical step is to convert this text into a format that the robot can understand and execute. This usually involves two sub-steps:

1.  **Natural Language Understanding (NLU)**: An NLU component (often another LLM or a specialized NLP model) analyzes the transcribed text to identify the user's **intent** (e.g., "navigate," "pick up," "find") and extract relevant **entities** and **parameters** (e.g., "kitchen," "cup," "red box").

2.  **Action Mapping**: The extracted intent and parameters are then mapped to specific robot actions. In a ROS 2 system, these actions typically correspond to:
    *   **ROS 2 Actions**: For long-running, goal-oriented tasks (e.g., `NavigateToPose`, `PickObject`).
    *   **ROS 2 Services**: For single request-response operations (e.g., `GetBatteryStatus`).
    *   **ROS 2 Topics**: For publishing simple commands (e.g., velocity commands to a mobile base).

**Example: Voice "Move forward" → ROS 2 command**

Let's imagine a user says, "Robot, move forward by two meters."

*   **Whisper**: Transcribes "Robot, move forward by two meters."
*   **NLU (LLM)**: Analyzes the text, extracts intent: `navigate`, parameter: `distance=2m`, direction: `forward`.
*   **Action Mapping**: Translates this into a ROS 2 command. For a mobile robot, this might involve:
    *   Sending a goal to a `NavigateToPose` action client, with the target pose being 2 meters directly in front of the robot.
    *   Alternatively, publishing a `geometry_msgs/msg/Twist` message to a `/cmd_vel` topic for a short duration, with linear `x` velocity set to a positive value.

This entire sequence, from a human's spoken word to a robot's physical movement, forms the Voice-to-Action pipeline, demonstrating a fundamental aspect of VLA.
