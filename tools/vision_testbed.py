#!/usr/bin/env python3
"""
PiCar-X Vision Testbed
Test vision prompts and models without running the car.

Usage:
    # Test with live camera, default task
    python3 tools/vision_testbed.py

    # Test with a saved image file
    python3 tools/vision_testbed.py --image /tmp/garage.jpg

    # Test a specific task
    python3 tools/vision_testbed.py --task "find the red Coca-Cola aluminum can"

    # Test a specific model only
    python3 tools/vision_testbed.py --model qwen2.5vl:latest

    # Run continuously every 3 seconds (live camera)
    python3 tools/vision_testbed.py --loop --interval 3

    # Save current camera frame for offline testing
    python3 tools/vision_testbed.py --save /tmp/test_frame.jpg

    # Verbose — show full model responses
    python3 tools/vision_testbed.py --verbose
"""

import sys
import os
import time
import base64
import argparse
import urllib.request
import requests
from datetime import datetime

# ── Load config ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import (
        PI_IP, CAMERA_PORT, AGENT_PORT,
        OLLAMA_IP, OLLAMA_PORT, OLLAMA_MODEL,
        VISION_TIMEOUT, TARGET_CONFIRM_NEEDED,
    )
    OLLAMA_URL = f"http://{OLLAMA_IP}:{OLLAMA_PORT}/api/generate"
    CAMERA_URL = f"http://{PI_IP}:{CAMERA_PORT}/mjpg"
    AGENT_URL  = f"http://{PI_IP}:{AGENT_PORT}"
    print(f"Config loaded. Ollama: {OLLAMA_URL}")
except ImportError:
    print("config.py not found — using defaults")
    OLLAMA_URL            = "http://localhost:11434/api/generate"
    CAMERA_URL            = "http://YOUR_PI_IP:9000/mjpg"
    AGENT_URL             = "http://YOUR_PI_IP:8000"
    OLLAMA_MODEL          = "qwen2.5vl:latest"
    VISION_TIMEOUT        = 30
    TARGET_CONFIRM_NEEDED = 2

# ── Default models to test if none specified ───────────────────────────────────
DEFAULT_MODELS = [OLLAMA_MODEL]

# ── Frame capture ──────────────────────────────────────────────────────────────
def capture_frame(url=None):
    """Capture a single JPEG frame from the Pi camera stream."""
    url = url or CAMERA_URL
    try:
        req  = urllib.request.urlopen(url, timeout=5)
        data = b''
        while True:
            chunk = req.read(1024)
            data += chunk
            start = data.find(b'\xff\xd8')
            end   = data.find(b'\xff\xd9')
            if start != -1 and end != -1:
                req.close()
                return data[start:end+2]
    except Exception as e:
        print(f"  ❌ Camera error: {e}")
        return None

def capture_best_frame(n=3):
    """Capture N frames and return the largest (most detail)."""
    frames = []
    for i in range(n):
        frame = capture_frame()
        if frame:
            frames.append(frame)
        if i < n - 1:
            time.sleep(0.3)
    if not frames:
        return None
    best = max(frames, key=len)
    print(f"  📷 Captured {len(frames)}/{n} frames, best: {len(best):,} bytes")
    return best

def load_image(path):
    """Load image from file."""
    with open(path, 'rb') as f:
        data = f.read()
    print(f"  📁 Loaded image: {path} ({len(data):,} bytes)")
    return data

# ── Prompt builder ─────────────────────────────────────────────────────────────
def build_navigation_prompt():
    """Standard navigation prompt — no task."""
    return """You are a navigation assistant for a small robot car.
Look at this camera image and answer:
1. What obstacles or objects do you see?
2. Is the path ahead clear?
3. Which direction should the robot move?

Be concise. Answer in 2-3 sentences."""

def build_goal_prompt(task):
    """Goal detection prompt for a specific task."""
    return f"""Look carefully at this image and answer these questions:
1. What objects do you see on the floor or in the scene?
2. Do you see: {task}?
3. If yes, is it on the LEFT, CENTER, or RIGHT side of the image?
4. What direction should a robot move to get closer to it?

Be specific and concise."""

# ── Response parser ────────────────────────────────────────────────────────────
def parse_response(text, task):
    """
    Parse free-form model response to extract:
    - found: bool
    - where: left/center/right/unknown
    - hint: forward/left/right/stop/none
    - confidence: high/medium/low
    """
    text_lower = text.lower()

    # Extract meaningful task keywords
    stop_words = {'find', 'the', 'a', 'an', 'some', 'look', 'for', 'get',
                  'locate', 'search', 'seek', 'identify', 'spot'}
    task_words = [w for w in task.lower().split() if w not in stop_words]

    # Check if target was found
    found = any(word in text_lower for word in task_words)

    # Negative indicators override
    negative = any(phrase in text_lower for phrase in [
        'no ', 'not ', "don't", "doesn't", "can't", 'cannot',
        'unable', 'no sign', 'not visible', 'not present', 'not found'
    ])
    if negative and not any(phrase in text_lower for phrase in [
        'yes', 'i see', 'i can see', 'there is', 'visible', 'present'
    ]):
        found = False

    # Parse position
    where = "unknown"
    if   'left'   in text_lower: where = 'left'
    elif 'right'  in text_lower: where = 'right'
    elif any(w in text_lower for w in ['center', 'middle', 'front', 'straight']):
        where = 'center'

    # Derive navigation hint
    if found:
        hint = where if where != 'unknown' else 'forward'
    else:
        # No target — suggest exploration direction
        if   'left'  in text_lower: hint = 'left'
        elif 'right' in text_lower: hint = 'right'
        else:                        hint = 'forward'

    # Confidence
    high_conf = any(p in text_lower for p in [
        'yes', 'i can see', 'i see', 'there is', 'clearly', 'definitely',
        'confident', 'visible', 'present'
    ])
    confidence = 'HIGH' if (found and high_conf) else ('MEDIUM' if found else 'LOW')

    return {
        "found":      found,
        "where":      where,
        "hint":       hint,
        "confidence": confidence,
    }

# ── Model query ────────────────────────────────────────────────────────────────
def query_model(model, image_b64, prompt, timeout=None):
    """Query a vision model and return response + timing."""
    timeout = timeout or VISION_TIMEOUT
    start   = time.time()
    try:
        response = requests.post(OLLAMA_URL, json={
            "model":  model,
            "prompt": prompt,
            "images": [image_b64],
            "stream": False
        }, timeout=timeout)
        elapsed  = time.time() - start
        text     = response.json().get('response', '')
        return text, elapsed, None
    except Exception as e:
        elapsed = time.time() - start
        return '', elapsed, str(e)

# ── Display helpers ────────────────────────────────────────────────────────────
def print_separator(char='─', width=60):
    print(char * width)

def print_result(model, text, elapsed, parsed, error=None, verbose=False):
    print(f"\n  Model:      {model}")
    print(f"  Time:       {elapsed:.1f}s")

    if error:
        print(f"  ❌ Error:   {error}")
        return

    found_icon = "✅" if parsed["found"] else "❌"
    print(f"  Found:      {found_icon} {parsed['found']}")
    print(f"  Confidence: {parsed['confidence']}")
    print(f"  Where:      {parsed['where']}")
    print(f"  Hint:       {parsed['hint']}")

    if verbose:
        print(f"\n  Full response:")
        for line in text.split('\n'):
            print(f"    {line}")
    else:
        # Show first 150 chars
        preview = text[:150].replace('\n', ' ')
        if len(text) > 150:
            preview += '...'
        print(f"  Preview:    {preview}")

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='PiCar-X Vision Testbed — test vision models without running the car'
    )
    parser.add_argument('--image',    '-i', help='Path to image file (uses live camera if not set)')
    parser.add_argument('--task',     '-t', default='find the red Coca-Cola aluminum can, approximately 12cm tall',
                        help='Task description for goal detection')
    parser.add_argument('--model',    '-m', help='Specific model to test (default: all configured models)')
    parser.add_argument('--loop',     '-l', action='store_true', help='Run continuously')
    parser.add_argument('--interval', type=float, default=3.0, help='Loop interval in seconds')
    parser.add_argument('--save',     '-s', help='Save captured frame to this path')
    parser.add_argument('--verbose',  '-v', action='store_true', help='Show full model responses')
    parser.add_argument('--navigate', '-n', action='store_true', help='Use navigation prompt instead of goal prompt')
    parser.add_argument('--frames',   '-f', type=int, default=3, help='Number of frames to capture (best is used)')
    args = parser.parse_args()

    models = [args.model] if args.model else DEFAULT_MODELS
    task   = args.task

    print_separator('═')
    print("  PiCar-X Vision Testbed")
    print_separator('═')
    print(f"  Task:     {task}")
    print(f"  Models:   {models}")
    print(f"  Source:   {'File: ' + args.image if args.image else 'Live camera'}")
    print(f"  Prompt:   {'Navigation' if args.navigate else 'Goal detection'}")
    print_separator('═')

    # Track consecutive confirmations
    confirm_counts = {m: 0 for m in models}

    iteration = 0
    while True:
        iteration += 1
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"\n{'─'*60}")
        print(f"  Iteration {iteration}  [{ts}]")
        print('─'*60)

        # Load or capture image
        if args.image:
            try:
                image_data = load_image(args.image)
            except Exception as e:
                print(f"  ❌ Could not load image: {e}")
                break
        else:
            image_data = capture_best_frame(n=args.frames)
            if image_data is None:
                print("  ❌ Could not capture frame — is the Pi agent running?")
                if not args.loop:
                    break
                time.sleep(args.interval)
                continue

        # Save frame if requested
        if args.save:
            try:
                with open(args.save, 'wb') as f:
                    f.write(image_data)
                print(f"  💾 Frame saved to {args.save}")
                if not args.loop:
                    break
            except Exception as e:
                print(f"  ❌ Save failed: {e}")

        image_b64 = base64.b64encode(image_data).decode()

        # Build prompt
        if args.navigate:
            prompt = build_navigation_prompt()
        else:
            prompt = build_goal_prompt(task)

        # Query each model
        for model in models:
            text, elapsed, error = query_model(model, image_b64, prompt)

            if error:
                print_result(model, '', elapsed, {}, error=error, verbose=args.verbose)
                confirm_counts[model] = 0
                continue

            parsed = parse_response(text, task)
            print_result(model, text, elapsed, parsed, verbose=args.verbose)

            # Track confirmations
            if parsed["found"] and parsed["confidence"] in ("HIGH", "MEDIUM"):
                confirm_counts[model] += 1
                needed = TARGET_CONFIRM_NEEDED
                print(f"  Confirmations: {confirm_counts[model]}/{needed}")
                if confirm_counts[model] >= needed:
                    print(f"\n  🎯 TARGET CONFIRMED after {needed} consecutive detections!")
                    print(f"     → Robot should approach from the {parsed['where']}")
                    confirm_counts[model] = 0  # reset after reporting
            else:
                if confirm_counts[model] > 0:
                    print(f"  ⚠️  Confirmation reset (was {confirm_counts[model]})")
                confirm_counts[model] = 0

        if not args.loop:
            break

        print(f"\n  ⏱  Next query in {args.interval}s... (Ctrl+C to stop)")
        try:
            time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n\nStopped.")
            break

    print("\nDone.")

if __name__ == "__main__":
    main()
