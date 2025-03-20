import numpy as np
import matplotlib.pyplot as plt

# Parameter Initialization
initial_soc = 0.9
X = np.array([[initial_soc], [0], [0]])  # Initial State space [InitialSOC, V@R1C1, V@R2C2]
P0 = np.diag([0.1, 0.1, 0.1])            # Error Co-Variance for State Space Equation
Q = 1e-5                                  # Process Noise for State Equation
R = 5e-4                                  # Measurement Noise for Output Equation
VL = np.zeros(3601)                       # Terminal/Load Voltage
VL[0] = 4.2                               # Initial Terminal/Load Voltage
I = 50                                    # Constant Discharge Current of 50Amps
eff = 0.98                                # Coulombic Efficiency
Q0 = 50 * 3600                            # Nominal Capacity of Cell
delta_T = 1                               # Sampling time, 1 second
N = int(3600 / delta_T)                   # Total Number of Iterations

# Arrays to store results
X_sim = np.zeros((3, N+1))
X_sim[:, 0] = X.flatten()
UL = np.zeros(N+1)
UL[0] = VL[0]

# Function to calculate battery parameters based on SoC
def calc_battery_params(soc):
    R_int = 0.00573 - 0.03427*soc**1 + 0.1455*soc**2 - 0.32647*soc**3 + \
            0.41465*soc**4 - 0.28992*soc**5 + 0.09353*soc**6 - 0.00634*soc**7
    
    R1 = 0.01513 - 0.18008*soc**1 + 1.05147*soc**2 - 3.27616*soc**3 + \
         5.79793*soc**4 - 5.81819*soc**5 + 3.08032*soc**6 - 0.66827*soc**7
    
    C1 = 47718.90713 - 1.00583e6*soc**1 + 9.2653e6*soc**2 - 3.91088e7*soc**3 + \
         8.85892e7*soc**4 - 1.11014e8*soc**5 + 7.22811e7*soc**6 - 1.90336e7*soc**7
    
    R2 = 0.01513 - 0.18008*soc**1 + 1.05147*soc**2 - 3.27616*soc**3 + \
         5.79793*soc**4 - 5.81819*soc**5 + 3.08032*soc**6 - 0.66827*soc**7
    
    C2 = 47718.90713 - 1.00583e6*soc**1 + 9.2653e6*soc**2 - 3.91088e7*soc**3 + \
         8.85892e7*soc**4 - 1.11014e8*soc**5 + 7.22811e7*soc**6 - 1.90336e7*soc**7
    
    Uocv = -23.60229*soc**7 + 141.34077*soc**6 - 314.92282*soc**5 + \
           345.34531*soc**4 - 200.15462*soc**3 + 60.21383*soc**2 - 7.88447*soc + 3.2377
    
    return R_int, R1, C1, R2, C2, Uocv

# Function to calculate system matrices A and B
def calc_system_matrices(R1, C1, R2, C2, delta_T, eff, Q0):
    tau1 = R1 * C1
    tau2 = R2 * C2
    
    A1 = 1
    A2 = np.exp(-delta_T / tau1)
    A3 = np.exp(-delta_T / tau2)
    
    A = np.array([[A1, 0, 0], 
                 [0, A2, 0], 
                 [0, 0, A3]])
    
    B1 = -delta_T * eff / Q0
    B2 = R1 * (1 - np.exp(-delta_T / tau1))
    B3 = R2 * (1 - np.exp(-delta_T / tau2))
    
    B = np.array([[B1], [B2], [B3]])
    
    return A, B, tau1, tau2

# Function to calculate Jacobian for OCV-SOC relationship
def calc_ocv_jacobian(soc):
    return -23.60229*7*soc**6 + 141.34077*6*soc**5 - 314.92282*5*soc**4 + \
           345.34531*4*soc**3 - 200.15462*3*soc**2 + 60.21383*2*soc - 7.88447

# Function to calculate Jacobian for R_int-SOC relationship
def calc_rint_jacobian(soc):
    return -0.03427*1 + 0.1455*2*soc - 0.32647*3*soc**2 + \
           0.41465*4*soc**3 - 0.28992*5*soc**4 + 0.09353*6*soc**5 - \
           0.00634*7*soc**6

# Simulation Loop
for t in range(1, N+1):
    SOC = X_sim[0, t-1]
    R_int, R1, C1, R2, C2, Uocv = calc_battery_params(SOC)
    A, B, tau1, tau2 = calc_system_matrices(R1, C1, R2, C2, delta_T, eff, Q0)
    
    # State update with process noise
    X_sim[:, t] = (A @ X_sim[:, t-1].reshape(-1, 1) + B * I + np.sqrt(Q) * np.random.randn(3, 1)).flatten()
    V_RC1 = X_sim[1, t]
    V_RC2 = X_sim[2, t]
    
    # Output Equation with measurement noise
    UL[t] = Uocv - V_RC1 - V_RC2 - I * R_int + np.sqrt(Q) * np.random.randn()

# Kalman Filter Implementation
X_update = np.zeros((3, N+1))
X_update[:, 0] = np.array([initial_soc, 0, 0])
X_predict = np.zeros((3, N+1))

for t in range(1, N+1):
    SOC = X_update[0, t-1]
    
    # Calculate battery parameters
    R_int, R1, C1, R2, C2, _ = calc_battery_params(SOC)
    A, B, tau1, tau2 = calc_system_matrices(R1, C1, R2, C2, delta_T, eff, Q0)
    
    # State Prediction Step
    X_predict[:, t] = (A @ X_update[:, t-1].reshape(-1, 1) + B * I).flatten()
    SOC_predict = X_predict[0, t]
    V_RC1_predict = X_predict[1, t]
    V_RC2_predict = X_predict[2, t]
    
    # Calculate predicted output using predicted SOC
    _, _, _, _, _, Uocv_predict = calc_battery_params(SOC_predict)
    
    # Calculate R_int based on predicted SOC for better accuracy
    R_int_predict, _, _, _, _, _ = calc_battery_params(SOC_predict)
    VL_predict = Uocv_predict - V_RC1_predict - V_RC2_predict - I * R_int_predict
    
    # Error Covariance Prediction
    P1 = A @ P0 @ A.T + np.diag([Q, Q, Q])
    
    # Jacobian Calculation (Improved)
    Cu = calc_ocv_jacobian(SOC_predict)
    dR_int = calc_rint_jacobian(SOC_predict)
    C = np.array([[Cu - I * dR_int, -1, -1]])
    
    # Kalman Gain Calculation
    K = (P1 @ C.T) / (C @ P1 @ C.T + R)
    
    # Update State Estimation
    X_update[:, t] = X_predict[:, t] + (K * (UL[t] - VL_predict)).flatten()
    
    # Update Error Covariance
    P0 = P1 - K @ C @ P1

# Plotting
plt.figure(figsize=(10, 6))
plt.grid(True)
plt.plot(range(N+1), X_update[0, :], '.', label='SoC Update (EKF)')
plt.plot(range(N+1), X_sim[0, :], 'g', label='SoC Without EKF')
plt.plot(range(N+1), X_predict[0, :], 'r', label='SoC Predict')
plt.xlabel('Time (s)')
plt.ylabel('State of Charge')
plt.title('Battery State of Charge Estimation')
plt.legend(loc='best')
plt.tight_layout()
plt.show()
