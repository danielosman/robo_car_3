# VLM bounding boxes to a floor map

Two chat answers pasted by the user (2026-10-17), kept as they came. Their
claims (FOV numbers, model speeds, the 9 cm height) are not checked against
this robot.

---

## 1. Multi-agent orchestration vs. detection + geometry

Yes, a multi-agent LangChain orchestration is massive overkill for this specific task—and it will likely introduce high latency, cascading errors, and heavy API/token costs without solving the core geometric problem.

Large Language Models (LLMs) and Vision-Language Models (VLMs) are notoriously bad at precise pixel-to-coordinate tracking and spatial mapping out of the box. Passing text lists back and forth between agents doesn't give them the spatial "spatial awareness" needed to translate a 3D perspective from a 160x120 camera view onto a top-down 2D floor plan.

Instead of an expensive multi-agent loop, a much cleaner, faster, and more accurate architecture splits the problem into two distinct, specialized layers: Vision/Perception and Geometric Projection.

### The Optimized Architecture (No Agents Needed)

```
[ 160x120 Image ] ──> ( Step 1: VLM/YOLO ) ──> [ Labels + 2D Screen Bounding Boxes ]
                                                             │
                                                     ( Step 2: Spatial Matrix )
                                                             │
                                                             ▼
                                                [ Top-Down 2D Map Coordinates ]
```

### Step 1: Detect Objects & Screen-Space Bounding Boxes

Instead of a raw text list, you need your vision layer to output the objects and their coordinates relative to the image frame (Screen Bounding Boxes). You have two great local options here:

**The Native VLM Route (Gemma 4 26B A4B / 31B):**
Gemma 4 natively supports object detection prompting. You can pass the image with a strict system prompt: "Identify all macro objects (desk, shelf, chair, table) and output their 2D bounding boxes in normalized screen coordinates [ymin, xmin, ymax, xmax] format."
Result: You get a clean JSON or structured text response like `{"object": "chair", "box": [40, 20, 80, 50]}` in a single forward pass.

**The Traditional Edge Route (YOLOv8 / YOLOv10 Nano):**
If your target objects are standard furniture items, a tiny local model like YOLOv8 Nano (trained on the COCO dataset) will process your 160x120 frame in sub-10 milliseconds at virtually 0% VRAM cost, outputting highly accurate bounding box arrays. This is vastly superior to an LLM for raw detection.

### Step 2: The Geometric Projection (Homography / Inverse Perspective)

An LLM agent cannot accurately guess where a bounding box goes on a 2D floor plan map just by looking at a picture, because it doesn't know the camera's height, angle, or distance from the wall.

Instead of an agent, use a simple Python deterministic script to handle the translation:

- **The Math:** If your camera position is fixed (e.g., a security camera or a robot's head at a set height), you can use a Homography Matrix or Inverse Perspective Mapping (IPM) via OpenCV in Python.
- **The Action:** The script takes the bottom edge of the bounding box provided by your detection step (which represents where the furniture touches the floor) and mathematically projects it directly onto your top-down 2D map grid.

### Why this beats the Multi-Agent Approach

| Feature | Multi-Agent LangChain Loop | Optimized Pipeline (VLM + Geometry) |
| :-- | :-- | :-- |
| Latency | 15 - 30+ seconds per frame (Multiple VLM calls, prompt generation, parsing). | < 100 milliseconds (Single VLM/YOLO inference + instantaneous math matrix). |
| Accuracy | Poor. LLMs hallucinate coordinate scaling and misinterpret spatial depth from text prompts. | High. Relies on fixed camera geometry and precise math for coordinate plotting. |
| Compute Cost | Devours VRAM/Tokens (Multiple agents analyzing images and writing spatial reasoning). | Ultra-lightweight (VLM handles vision only; CPU handles mapping math). |
| Fragility | High risk of parsing failure if Agent 1 formats the list slightly differently than expected. | Predictable, deterministic data pipeline. |

### Summary of What to Implement

1. Use Gemma 4 26B A4B (or YOLO if you want maximum speed) to output a structured JSON array containing the furniture labels and their pixel boundaries within the photo.
2. Pass those pixel coordinates to a Python script that uses `cv2.perspectiveTransform()` to project the object's ground placement onto your 2D grid map.

To choose the right approach for Step 2, is the camera fixed in a permanent position looking at the room, or is it mounted on a mobile robot that is moving around?

---

## 2. Arducam HM0360 at 9 cm: box rotation and a common-sense check

Knowing you are using the Arducam HM0360 changes everything. The stock lens on this module has a Diagonal FOV (DFOV) of 64°. Because it uses a standard 4:3 aspect ratio sensor, we can calculate your exact working parameters: [1, 2, 3]

- Horizontal FOV (HFOV): ~51.2°
- Vertical FOV (VFOV): ~38.4°

Since your camera is extremely low to the ground (9cm), axis-aligned bounding boxes (AABBs) and object rotation present distinct geometric constraints that your validation agent idea can directly optimize.

### Part 1: Is the parallel (Axis-Aligned) bounding box a problem?

Yes, it is a major problem if you rely on the raw width of the bounding box to size the object, but it is not a problem for finding its distance.

Because traditional detectors output boxes parallel to the image edges, when an object (like a long rectangular table or rectangular shelf) is rotated at an angle relative to the robot, its 2D bounding box artificially inflates. A diagonal table creates a giant square box filled mostly with background floor space. [4, 5]

```
   Rotated Object Map:             What the Detector Sees (Parallel AABB):
       ┌───┐                          ┌─────────┐
       │   │                          │ ░ ┌───┐ │  <-- Box width expands
       └───┘                          │ ░ │   │ │      artificially to encompass
     (Rotated)                        └─────────┘      the diagonal footprint.
```

**Who will determine the rotation?**
An axis-aligned VLM or YOLO cannot determine rotation from a 2D bounding box alone. You have two options to solve this:

- **The Math-Geometric Fix (Recommended for your 2D map):** Your projection math should completely ignore the width/sides of the bounding box. Instead, calculate distance using only the lowest center pixel of the bounding box (where the object meets the floor). By plotting this single contact point onto your 2D map across consecutive video frames as the robot moves, your mapping software (like a local SLAM or Occupancy Grid) will naturally trace out the shape and rotation of the obstacle over time.
- **The Vision Fix (Oriented Bounding Boxes):** If you swap your vision pipeline to an OBB (Oriented Bounding Box) model like YOLOv8-OBB, the model learns to output 5 values: `[x, y, w, h, angle]`. The model itself will tell you the exact rotation angle in degrees. [6, 7]

### Part 2: Is your "Estimation + Common Sense Verification Agent" overkill?

No, this part of your idea is brilliant and highly practical, but it should be structured as a heuristic filter rather than an expensive multi-agent LangChain loop.

Because your camera is at 9cm, a tiny 2-pixel fluctuation at the bottom of the image can cause your math script to jump from estimating a table distance of 1.5 meters to 2.5 meters. This causes the projected size of the object on your map to constantly expand, shrink, or teleport.

**How to build the "Common Sense" layer efficiently:**
Instead of prompting an LLM agent for every single frame (which creates heavy latency), hardcode a local Python dictionary of physical constraints (the "common sense") and let a light LLM handle parsing exceptions or room-level semantics.

```python
# Local "Common Sense" Heuristic Layer
FURNITURE_CONSTRAINTS = {
    "chair": {"min_w_cm": 35, "max_w_cm": 65, "max_h_cm": 110},
    "table": {"min_w_cm": 60, "max_w_cm": 240, "max_h_cm": 80},
    "shelf": {"min_w_cm": 40, "max_w_cm": 150, "max_h_cm": 210}
}

def verify_dimensions(label, calculated_width, calculated_length):
    if label in FURNITURE_CONSTRAINTS:
        limits = FURNITURE_CONSTRAINTS[label]
        # If the projection claims a chair is 1.5 meters wide, reject or cap it
        if calculated_width > limits["max_w_cm"]:
            return False, "Physically impossible dimensions for a " + label
    return True, "Valid"
```

### The Ideal Hybrid Workflow

1. **Perception:** Run Gemma 4 26B A4B or YOLO to get the label and the raw bottom-center pixel.
2. **Deterministic Projection:** Use your HM0360's 38.4° VFOV and 9cm height to mathematically calculate the rough coordinate on your map.
3. **The Verification Agent (Your Idea):** Pass the calculated spatial coordinates and label to a structured data validator. If the math says a "chair" is 2 meters wide because the bounding box is parallel/inflated, the validation script steps in, checks the dictionary constraints, and automatically scales the bounding box down on your map to a standard 50cm x 50cm placeholder footprint instead.

Are you planning to map the room statically (taking one photo at a time while stopped), or does the robot need to process this data dynamically on the fly while it is actively driving?

**Sources**

1. https://www.arducam.com
2. https://docs.arducam.com
3. https://www.arducam.com
4. https://developer.nvidia.com
5. https://github.com
6. https://blog.savant-ai.io
7. https://medium.com
