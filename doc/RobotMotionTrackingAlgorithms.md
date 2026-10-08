# **Robot Motion Tracking Algorithms**

[https\://gemini.google.com/app/606bf679a96d089d](https://gemini.google.com/app/606bf679a96d089d)

*User prompt: i am building a robot that has to track movement by rotating in the direction of movement \- i have two ways to detect movement \- via a camera and via a distance sensor that measures distances in a 8x8 grid. What algorithms would you recommend. i think there are already algorithms to detect movement in images \- if yes then should i just give a color to evey cell of the vl53 sensor based on the distance measurement and use the same algorithm for the vl53 sensor as for the image?*

Response: To track motion effectively with your robot, **you should not turn the VL53 8x8 depth grid into a pseudo-color image and run traditional computer vision image algorithms on it.**

While both sensors detect spatial data, standard 2D optical flow/image-tracking algorithms expect high pixel counts, spatial continuity, and visible light features. The VL53 sensor (like the VL53L5CX or VL53L8CX) gives you a tiny **8x8 matrix of physical distances (depth values)**, not light intensities. Treating depth values like 2D colors throws away key 3D information and causes false triggers when objects move relative to the background.

The recommended algorithms for both sensors, as well as an approach to combine them, are detailed below.

## **1\. Camera Algorithms (High Resolution, Light/Texture-Based)**

For tracking movement direction to rotate your robot, rely on standard **Optical Flow** or **Object Tracking**:

> * **Dense Optical Flow (Farneback or Lucas-Kanade):** Calculates motion vectors across the entire frame. Averaging these motion vectors gives you the overall direction of movement (e.g., net shift to the left or right).  
> * **Sparse Feature Tracking (Shi-Tomasi Corners \+ Lucas-Kanade):** Tracks specific high-contrast points frame-to-frame. Faster to compute on low-power hardware (like a Raspberry Pi or microcontroller with a hardware accelerator).  
> * **Background Subtraction \+ Contours (MOG2 or KNN):** Great if the robot is stationary while scanning. Subtracts the static background, isolates moving blobs, finds their contours, and tracks the blob's centroid displacement over time.

## **2\. VL53 (8x8 ToF) Algorithms (Low Resolution, 3D Distance-Based)**

Instead of converting distance to color, treat the 8x8 matrix as a **3D Point Cloud or a Temporal Difference Grid**:

> * **Temporal Distance Delta (Simplest & Fast):** Subtract the current 8x8 grid from the previous frame (*D*diff​\=*D*current​−*D*previous​).  
  * Negative values indicate an object moving **closer**.  
  * Positive values indicate an object moving **away**.  
  * Cells where ∣*D*diff​∣\>threshold mark active motion. Calculate the **Center of Mass (Centroid)** of these active cells across the 8x8 grid to determine if the movement is shifting left, right, up, or down.  
> * **3D Point Tracking / Grid Projection:** Convert each of the 64 zone readings (*xgrid*​,*ygrid*​,*Zdistance*​) into approximate (*X*,*Y*,*Z*) physical coordinates using the sensor's Field of View (FOV). Track how the minimum depth point moves in real physical space across frames.  
> * **Rule-Based Column Averaging:** Group the 8x8 grid into 8 columns. Average the distance in each column. If column 1 drops in distance first, then column 2, then column 3, the target is moving from left to right.

## **Why Color-Mapping the VL53 Is Problematic**

> 1. **Information Loss:** A camera measures light intensity; a ToF sensor measures physical *Z*\-distance. Converting distance to an 8-bit color map discards raw units (millimeters) and forces continuous physical space into artificial color steps.  
> 2. **Resolution Mismatch:** Algorithms like OpenCV's calcOpticalFlowFarneback rely on pixel gradients across hundreds of pixels. On an 8x8 grid (64 total "pixels"), image gradients break down, producing heavy noise and inaccurate vectors.  
> 3. **Background Disruption:** If a target moves sideways across the 8x8 grid, background cells get revealed. In an image algorithm, the edge transition will look like motion in the opposite direction, creating ghost vectors.

## **3\. Recommended Fusion Strategy (Camera \+ VL53)**

Combining both sensors yields a robust tracking pipeline:

┌──────────────┐     High Resolution / Direction  
│    Camera    ├─────────────────────────────────┐  
└──────────────┘                                 ▼  
                                       ┌──────────────────┐     Rotational Motor  
                                       │ Decision / Fusion├────► Command (Pan/Tilt)  
                                       └──────────────────┘  
┌──────────────┐     Low Latency / Distance / ROI    ▲  
│  VL53 (8x8)  ├─────────────────────────────────┘  
└──────────────┘

> 1. **Trigger / ROI Selection via VL53:** Use the 8x8 ToF sensor as a lightweight "radar" to instantly detect *where* in 3D space movement occurred (e.g., "Motion detected in columns 6–8 within 1.5 meters").  
> 2. **Target Verification via Camera:** Crop the camera frame to the region corresponding to the VL53's detection zone (Region of Interest / ROI).  
> 3. **Compute Direction & Track:** Run optical flow or centroid tracking on that cropped camera section to calculate the precise angular error, then command your motor to rotate toward the target.

---

*User prompt: i want to do these calculations on a pico 2 \- doable?*

Response: Yes, running these calculations on a **Raspberry Pi Pico 2** (powered by the **RP2350** chip) is **100% doable and well within its hardware capabilities**.

## **Pico 2 Performance & Feasibility**

| Processing Task | Pico 2 Feasibility | Performance & Implementation Notes |
| :---- | :---- | :---- |
| **VL53 8x8 Grid Calculations** | **Trivial** | Processing a 64-element matrix for delta tracking or centroid calculation requires minimal CPU cycles. You can easily process this at the full 15–60 Hz sensor refresh rate. |
| **Camera Processing (QVGA / QQVGA)** | **Doable with limits** | The Pico 2 lacks a hardware video pipeline, so full 1080p OpenCV is out. However, downsampled frames (e.g., 160×120 or 80×60 grayscale) can be read via PIO/SPI and processed using lightweight feature-tracking or optical flow in C/C++. |
| **Sensor Fusion & Motor Control** | **Trivial** | Low latency, hard real-time execution across the dual Cortex-M33 (or Hazard3 RISC-V) cores running up to 150 MHz. |

## **Key Hardware Advantages of Pico 2 for This Architecture**

> 1. **Dual Arm Cortex-M33 Cores @ 150 MHz (with DSP/FPU):**  
   * Dedicated hardware Floating Point Units (FPU) mean vector math, Kalman filtering matrix multiplications, and coordinate geometry happen almost instantly.  
   * **Dual Core Split:** Use **Core 0** for reading sensors & motor control loops, and **Core 1** dedicated solely to processing image frames and depth deltas.  
> 2. **520 KB SRAM:**  
   * Ample memory for holding multiple 160×120 8-bit frame buffers, the 8x8 depth matrices, and scratchpad memory for feature calculations.  
> 3. **Programmable I/O (PIO):**  
   * Essential for interfacing directly with camera sensor modules (like an OV7670 or HM01B0) without burdening the main CPU cores with byte-level clocking.

## **Recommended Software Stack**

> * **Language:** Use **C/C++ (Pico SDK)** or **Rust**. MicroPython/CircuitPython will be too slow for camera-based optical flow on microcontrollers.  
> * **Computer Vision Library:** Use **PicoVision** or lightweight C libraries like **pico-cam** / **OpenMV algorithms** port.  
> * **Matrix / Fusion Math:** Use **ARM CMSIS-DSP** (pre-optimized for Cortex-M processors) for fast matrix operations, Kalman filtering, and vector math.

---

*User prompt: and what about tracking movement when the robot is rotating in the same time? will the image movement detection algorithms work here as well \- will we get predicted movement of different parts of the image? also what about noise \- both the camera has noisy pixels and the vl53 can return different results even if nothing moved on the scene.*

Response: When the robot rotates while tracking, standard movement detection will fail unless you explicitly compensate for the robot's own rotation (ego-motion). Both sensors will perceive the static world moving past them, and sensor noise will corrupt the readings.

Here is how ego-motion and noise affect your tracking, along with the algorithms used to fix them.

## **1\. Tracking During Rotation (Ego-Motion Compensation)**

When the robot rotates panning to the right at angular speed *ω*, static objects appear to slide across the camera frame and the ToF grid to the left.

### **Camera Image Compensation**

If you use basic optical flow while rotating, **every pixel will have a horizontal motion vector**. To isolate true target movement, compute the predicted pixel shift caused by rotation and subtract it:

vtarget​\=vobserved​−vrotation​

> * **For pure horizontal rotation (panning):** The expected pixel velocity across the camera frame is approximately:  
>   Δ*x*pixels​\=*p*size​*f*⋅*ω*⋅Δ*t*​  
>   *(where f is focal length, ω is rotational velocity, and psize​ is pixel size).*  
> * **Target Detection:** Any pixel vector where vobserved​≈vrotation​ belongs to the static background. Targets moving independently will stand out as residual vectors (vtarget​\=0).

### **VL53 Depth Grid Compensation**

When the robot turns right by 1 column (e.g., 45∘/8≈5.6∘ per column):

> * Shift the previous 8×8 matrix to the left by 1 column.  
> * Subtract the shifted previous frame from the new current frame.  
> * Otherwise, static walls passing through the sensor field will trigger false positive depth changes.

## **2\. Handling Sensor Noise**

### **VL53 ToF Noise Sources & Fixes**

ToF sensors suffer from shot noise, multipath reflections, low-reflectivity target dropouts, and random distance jumps (±10–30 mm).

> 1. **Exponential Moving Average (EMA) / IIR Low-Pass Filter:** Smooth each cell's reading individually across time:  
>    *D*filtered​\[*i*\]=*α*⋅*D*raw​\[*i*\]+(1−*α*)⋅*D*filtered​\[*i*−1\]  
>    *(Set α≈0.2–0.4 to suppress high-frequency noise while retaining fast motion detection).*  
> 2. **Dynamic Noise Gate / Deadband:** Never trigger on small distance changes. Require a distance jump to exceed a deadband threshold:  
>    Active Cell if ∣*D*current​−*D*previous​∣\>Threshold(e.g., \>50 mm)  
> 3. **Spatial Median Filtering:** Apply a 3×3 median filter across the 8×8 grid to eliminate isolated single-cell spikes caused by bad laser returns.

### **Camera Noise Sources & Fixes**

Microcontroller cameras suffer from thermal sensor noise, compression artifacts, and motion blur during rotation.

> 1. **Gaussian Blur Pre-processing:** Blur image frames with a 3×3 kernel before computing optical flow to suppress pixel-level thermal noise.  
> 2. **Magnitude Thresholding:** Ignore small feature vectors (∣v∣\<threshold). Noise produces small 1–2 pixel jitter vectors, whereas true motion produces longer vectors.  
> 3. **RANSAC (Random Sample Consensus):** When fitting a motion model to feature vectors across the frame, use RANSAC to reject outlier vectors caused by noisy pixels or local lighting changes.

## **3\. Practical State Machine Strategy for Pico 2**

To keep execution fast and clean on the Pico 2, structure your control pipeline as follows:

┌────────────────────────────────────────────────────────┐  
│ 1\. Read Motor Encoders / IMU (Get Angular Velocity ω)  │  
└───────────────────────────┬────────────────────────────┘  
                            │  
                            ▼  
┌────────────────────────────────────────────────────────┐  
│ 2\. Predict Background Motion                           │  
│    • Camera: Expected horizontal pixel shift v\_rot     │  
│    • VL53: Column index shift based on Δθ              │  
└───────────────────────────┬────────────────────────────┘  
                            │  
                            ▼  
┌────────────────────────────────────────────────────────┐  
│ 3\. Filter & Subtract Ego-Motion                        │  
│    • VL53: Apply median filter \+ subtract shifted frame│  
│    • Camera: Apply Gaussian blur \+ subtract v\_rot      │  
└───────────────────────────┬────────────────────────────┘  
                            │  
                            ▼  
┌────────────────────────────────────────────────────────┐  
│ 4\. Extract Residual Motion                             │  
│    • Centroid of non-zero residual vectors \= Target    │  
└────────────────────────────────────────────────────────┘

---

*User prompt: when i am building a wheeled robot that i mentioned \- should i use ROS? why? it seems there is little i can gain \- i dont want to be stuck to one way of doing things*

Response: **No, you should not use ROS for this project.** Given your architecture and goals, introducing ROS (or Micro-ROS) will add significant setup friction and runtime bloat while offering very little practical benefit.

Your instinct is correct: ROS is **not** a set of robotics algorithms, but rather a communications middleware (a system for pass-through message broker networks).

## **Why ROS Doesn't Make Sense for Your Pico 2 Setup**

| Consideration | Bare-Metal / Native Pico C SDK | Micro-ROS (ROS 2 on Microcontrollers) |
| :---- | :---- | :---- |
| **Footprint & Overhead** | Light (\~few KB RAM/Flash), zero serialization overhead. | High memory footprint, requires XRCE-DDS middleware layer and agent nodes. |
| **Setup Friction** | Single CMake project in Pico SDK or C++. Flash and run. | Requires building Docker containers, cross-compilers, and running a ROS 2 Agent host PC. |
| **Ego-Motion Latency** | Deterministic, microsecond-level direct hardware loops on RP2350. | Message passing serialization can add jitter across serial/USB transport buffers. |
| **Flexibility / Vendor Lock** | Complete control; pure C/C++ libraries can be ported anywhere. | Ties you to ROS message structures (rcl, rmw) and ecosystem conventions. |

## **What ROS Is Actually Good For (And Why You Don't Need It Yet)**

ROS excels when building large, multi-computer, highly complex robots where dozens of independent modules need to talk over a network:

> 1. **Mapping & Path Planning (SLAM / Nav2):** If your robot needed to build a 2D map of a house, navigate around obstacles, and plan paths through doorways.  
> 2. **Standardized Hardware Interfaces:** If you were switching between 5 different LiDAR models and wanted a single software interface to work with all of them without changing code.  
> 3. **Multi-Node Networking:** If you had a Linux Single-Board Computer (like a Jetson or Pi) handling heavy AI models and streaming data over Ethernet to multiple microcontrollers handling individual limbs or wheels.

## **How to Stay Flexible Without ROS**

To avoid getting "stuck in one way of doing things," build a **modular C++ architecture** directly on the Pico 2:

                  ┌───────────────────────────────┐  
                  │        Hardware Layer         │  
                  │ (OV7670 Driver, VL53 Driver)  │  
                  └──────────────┬────────────────┘  
                                 │ Clean Structs (Frame, Matrix)  
                                 ▼  
                  ┌───────────────────────────────┐  
                  │       Processing Layer        │  
                  │ (Filtering, Ego-Motion Sub)  │  
                  └──────────────┬────────────────┘  
                                 │ Motion Vectors  
                                 ▼  
                  ┌───────────────────────────────┐  
                  │       Controller Layer        │  
                  │  (State Machine, Motor PID)   │  
                  └───────────────────────────────┘

> * **Separate Sensor Drivers from Math:** Write your optical flow and ToF subtraction logic to operate on plain C arrays/structs, completely agnostic of how the hardware delivered the data.  
> * **Use Custom Serial Packets for Telemetry:** If you want to visualize data on your PC (e.g., using Python or PlotJuggler), stream simple JSON or binary buffers over UART/USB.

**Bottom Line:** Stick to bare-metal C/C++ on the Pico 2\. If your robot eventually evolves to require autonomous SLAM, indoor navigation, or heavy neural networks on an external Pi/PC, you can easily wrap your existing C++ code into a ROS 2 package at that point.

---

