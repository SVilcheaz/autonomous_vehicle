import os
import stat
import time
import openwakeword
openwakeword.utils.download_models()
from openwakeword.model import Model
import sounddevice as sd
import numpy as np
from datetime import datetime

PIPE_PATH = "/tmp/wake_word_pipe"

# Point to your .onnx files
MODEL_PATHS = [
    "models/onnx/follow_me.onnx",
    "models/onnx/three_sixty.onnx",
    "models/onnx/autopilot.onnx",
    "models/onnx/full_stop.onnx",
]

THRESHOLD = 0.1
CHUNK = 1280  # 80ms at 16kHz

# Load all models in one shot
model = Model(
    wakeword_models=MODEL_PATHS,
    inference_framework="onnx"
)

if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)

print("Loaded models:", list(model.models.keys()))
print(f"Waiting for reader on {PIPE_PATH}...")
pipe_fd = os.open(PIPE_PATH, os.O_WRONLY)
print(f"Reader connected. Listening... (threshold={THRESHOLD})\n")

COOLDOWN_S = 1.5  # seconds before the same keyword can fire again

last_sent = {}  # keyword -> timestamp of last dispatch

def audio_callback(indata, frames, time_info, status):
    if status:
        print("Audio status:", status)
    
    pcm = np.frombuffer(indata, dtype=np.int16).flatten()
    predictions = model.predict(pcm)
    
    now = time.monotonic()
    
    for keyword, score in predictions.items():
        if score >= THRESHOLD:
            last = last_sent.get(keyword, 0)
            if now - last < COOLDOWN_S:
                continue  # Still in cooldown, suppress
            
            last_sent[keyword] = now
            ts = datetime.now().strftime("%H:%M:%S")
            bar = "█" * int(score * 20)
            print(f"[OWW] [{ts}] '{keyword}' detected! {bar} ({score:.2f})")
            os.write(pipe_fd, f"{keyword}\n".encode())

try:
    with sd.RawInputStream(
        samplerate=16000,
        channels=1,
        dtype="int16",
        blocksize=CHUNK,
        callback=audio_callback
    ):
        while True:
            sd.sleep(100)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    os.close(pipe_fd)