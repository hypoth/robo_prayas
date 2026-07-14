# Differential Drive Turning Calculations & Lookup Table

This document details the mathematical framework, calculations, and lookup tables required to make a two-wheel differential drive robot execute precise in-place turns.

---

## 1. Mathematical Framework

To turn a differential drive robot on the spot, the wheels must rotate in opposite directions at identical speeds. The robot rotates around a central point located exactly halfway between the two wheels.

### Variables & Constants
* **$\theta$ (Turn Angle):** Target body turn angle in degrees or radians.
* **$L$ (Track Width):** The distance between the center points of the two wheels.
* **$C_w$ (Wheel Circumference):** The distance a wheel travels in one full rotation.
* **$d$ (Wheel Diameter):** The diameter of the wheels ($C_w = \pi \cdot d$).

### Step-by-Step Formulas

#### Step A: Calculate Arc Distance per Wheel
Each wheel traces an arc of a circle whose radius is half the track width ($L/2$). The linear ground distance ($S$) each wheel must travel is:
$$S = \text{Turning Radius} \times \text{Turn Angle in Radians}$$
$$S = \frac{L}{2} \times \left(\theta^\circ \times \frac{\pi}{180^\circ}\right)$$

#### Step B: Calculate Required Wheel Rotations
To find the fractional rotations ($N$) needed to cover distance $S$:
$$N = \frac{\text{Distance to travel } (S)}{\text{Wheel Circumference } (C_w)}$$

#### Step C: Convert to Wheel Degrees
Multiply fractional rotations by $360^\circ$ to find the required individual motor rotation angle ($Deg_w$):
$$Deg_w = N \times 360^\circ = \theta^\circ \times \frac{\pi \cdot L}{C_w}$$

---

## 2. Robot Specifications

The physical parameters used for the localized calculations below are:
* **Track Width ($L$):** $100\text{ mm}$
* **Wheel Diameter ($d$):** $65\text{ mm}$
* **Wheel Circumference ($C_w$):** $\pi \times 65\text{ mm} \approx 204.20\text{ mm}$

---

## 3. Turn Angle Reference Examples

### 15-Degree Turn Example
* **Arc Distance ($S$):** $\frac{100}{2} \times \left(15^\circ \times \frac{\pi}{180^\circ}\right) = 13.09\text{ mm}$
* **Wheel Rotations ($N$):** $\frac{13.09\text{ mm}}{204.20\text{ mm}} = 0.0641\text{ rotations}$
* **Wheel Degrees ($Deg_w$):** $0.0641 \times 360^\circ = 23.08^\circ$

### 10-Degree Turn Example
* **Arc Distance ($S$):** $\frac{100}{2} \times \left(10^\circ \times \frac{\pi}{180^\circ}\right) = 8.73\text{ mm}$
* **Wheel Rotations ($N$):** $\frac{8.73\text{ mm}}{204.20\text{ mm}} = 0.0427\text{ rotations}$
* **Wheel Degrees ($Deg_w$):** $0.0427 \times 360^\circ = 15.38^\circ$

---

## 4. Comprehensive Turning Lookup Table

The values below are calculated for a **clockwise (right) turn on the spot**. For a counter-clockwise turn, invert the left and right wheel directional signs (+/-).

* **100 RPM** translates to $600^\circ/\text{sec}$ ($100 \times 360^\circ / 60\text{s}$).
* **50 RPM** translates to $300^\circ/\text{sec}$ ($50 \times 360^\circ / 60\text{s}$).

| Turn Angle (Body Degrees) | Left Wheel Rotation | Right Wheel Rotation | Linear Distance per Wheel | Time Taken at 100 RPM | Time Taken at 50 RPM |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0°** | 0.00° | 0.00° | 0.00 mm | 0.000 s | 0.000 s |
| **10°** | +15.38° | -15.38° | 8.73 mm | 0.026 s | 0.051 s |
| **20°** | +30.77° | -30.77° | 17.45 mm | 0.051 s | 0.103 s |
| **30°** | +46.15° | -46.15° | 26.18 mm | 0.077 s | 0.154 s |
| **40°** | +61.54° | -61.54° | 34.91 mm | 0.103 s | 0.205 s |
| **50°** | +76.92° | -76.92° | 43.63 mm | 0.128 s | 0.256 s |
| **60°** | +92.31° | -92.31° | 52.36 mm | 0.154 s | 0.308 s |
| **70°** | +107.69° | -107.69° | 61.09 mm | 0.179 s | 0.359 s |
| **80°** | +123.08° | -123.08° | 69.81 mm | 0.205 s | 0.410 s |
| **90°** | +138.46° | -138.46° | 78.54 mm | 0.231 s | 0.462 s |

---

## 5. Execution Logic
To turn the physical robot **clockwise**:
1. Drive the **Left Motor** forward.
2. Drive the **Right Motor** backward.
3. Halt both motors when the target wheel degrees or target time duration is reached.

