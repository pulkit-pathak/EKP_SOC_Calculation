# README: SoC Estimation using Extended Kalman Filter (EKF)

## **Project Overview**
This MATLAB script implements **State of Charge (SoC) estimation** for a battery using an **Extended Kalman Filter (EKF)**. The algorithm models the battery's electrical behavior using an equivalent circuit model (ECM) and applies the EKF to estimate the SoC in real time.

The script consists of two main parts:
1. **Simulation Loop:** Generates synthetic battery data based on an RC model.
2. **Kalman Filter Loop:** Applies EKF to estimate SoC based on noisy voltage measurements.

---

## **1. Files in This Project**
- `SoC_EKF.m`: Main MATLAB script for SoC estimation.
- `README.md`: Explanation of the script.
- `State_of_charge.jpg`: Output plot showing estimated SoC.

---

## **2. Model Description**
The system models the battery as a state-space system with three states:
- **SoC (State of Charge)**
- **Voltage across RC1 (V_{RC1})**
- **Voltage across RC2 (V_{RC2})**

### **State Equations**
The battery dynamics are modeled as:
\[
    X(k+1) = A \cdot X(k) + B \cdot I + W(k)
\]
where:
- **A** = State transition matrix
- **B** = Input matrix
- **I** = Current input
- **W(k)** = Process noise

### **Measurement Equation**
The measured terminal voltage is given by:
\[
    V_L = U_{OCV}(SoC) - V_{RC1} - V_{RC2} - I \cdot R_{int} + V(k)
\]
where:
- **U_{OCV}(SoC)** = Open-circuit voltage (OCV), a nonlinear function of SoC
- **R_{int}** = Internal resistance, also a nonlinear function of SoC
- **V(k)** = Measurement noise

---

## **3. Script Structure**
### **(a) Simulation Loop (Generates Battery Data)**
- Initializes system parameters (SoC, RC voltages, resistance values, etc.).
- Uses a state-space model to simulate voltage response under a constant discharge current.
- Stores noisy voltage measurements in `UL` to be used in the EKF.

### **(b) Kalman Filter Implementation**
- Initializes EKF parameters (state estimates, error covariance, process noise, measurement noise).
- Implements EKF prediction and update steps.
- Uses an improved measurement Jacobian that includes **both OCV and internal resistance derivatives**.
- Updates SoC estimates over time.

---

## **4. Key Equations Used in EKF**
### **Prediction Step**
\[
    X_{predict} = A \cdot X_{update} + B \cdot I
\]
\[
    P_{predict} = A \cdot P_{update} \cdot A^T + Q
\]

### **Update Step**
\[
    K = P_{predict} \cdot C^T \cdot (C \cdot P_{predict} \cdot C^T + R)^{-1}
\]
\[
    X_{update} = X_{predict} + K \cdot (UL - V_{predict})
\]
\[
    P_{update} = (I - K \cdot C) \cdot P_{predict}
\]

where:
- **K** = Kalman Gain
- **C** = Measurement Jacobian

---

## **5. How to Run the Script**
### **Requirements**
- MATLAB installed
- Basic understanding of state-space models and Kalman filtering

### **Running the Script**
1. Open `SoC_EKF.m` in MATLAB.
2. Run the script (`F5` or `Run` button).
3. View the output SoC plot and compare estimation accuracy.

---

## **6. Expected Output**
- The script generates a plot showing **SoC estimation over time**.
- It compares the estimated SoC (`X_update(1,:)`) against the actual SoC (`X(1,:)`).

---

## **7. Troubleshooting & Debugging**
### **Common Issues & Fixes**
1. **Incorrect SoC Estimates?**
   - Check measurement Jacobian `C` (ensure it includes derivative of \( R_{int} \)).
   - Verify process noise `Q` and measurement noise `R` values.

2. **Simulation & EKF Mismatch?**
   - Ensure the same state-space model is used in both loops.
   - Use `SOC_predict` consistently for computing `Uocv` and `R_int`.

3. **Kalman Gain Too High or Low?**
   - Adjust noise parameters (`Q`, `R`) for better tuning.

---

## **8. Future Improvements**
- **Dynamic Current Profile:** Instead of a constant 50A discharge, incorporate real driving cycles (e.g., WLTP, NEDC).
- **Adaptive Noise Estimation:** Implement an adaptive filter to tune `Q` and `R` dynamically.
- **SOC Correction using Coulomb Counting:** Combine Kalman Filter with a simple Coulomb counting model for improved accuracy.

---

## **9. References**
- [Kalman Filters for Battery Management](https://ieeexplore.ieee.org/document/7438630)
- [MATLAB Documentation on EKF](https://www.mathworks.com/help/control/ref/extendedkalmanfilter.html)
- [Battery Modeling Techniques](https://www.sciencedirect.com/science/article/pii/S0378775317303986)

---

## **10. Contact**
For questions or improvements, contact **[Pulkit Pathak]** at **pathakpulkit06@gmail.com**

