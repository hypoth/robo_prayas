# Robot Travel Distance Calculation

## Summary
* **Total Distance Traveled:** ~8.51 mm (0.85 cm)
* **Time Duration:** 25 ms
* **Motor Speed:** 100 RPM
* **Wheel Diameter:** 65 mm

---

## Step-by-Step Breakdown

### 1. Wheel Circumference
Find the distance covered in one full wheel rotation.
$$\text{Circumference} = \pi \times \text{Diameter}$$
$$\text{Circumference} = \pi \times 65\text{ mm} \approx 204.2035\text{ mm}$$

### 2. Rotations per Millisecond
Convert Revolutions Per Minute (RPM) to Revolutions Per Millisecond (rev/ms).
$$\text{Rotations per second} = \frac{100\text{ RPM}}{60\text{ seconds}} \approx 1.6667\text{ RPS}$$
$$\text{Rotations per millisecond} = \frac{1.6667\text{ RPS}}{1000\text{ ms}} \approx 0.001667\text{ rev/ms}$$

### 3. Total Distance
Multiply the speed by the target time frame (25 ms).
$$\text{Total Rotations in 25ms} = 0.001667\text{ rev/ms} \times 25\text{ ms} \approx 0.04167\text{ revolutions}$$
$$\text{Distance} = 204.2035\text{ mm} \times 0.04167\text{ revolutions} \approx \mathbf{8.51\text{ mm}}$$

