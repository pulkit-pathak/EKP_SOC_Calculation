%Projec Name : SoC calculation based on Extended Kalman Filter

close all;
clc;


%% Parameter Initialisation

X = [0.9; 0; 0];                                                                % Initial State space[InitialSOC, V@R1C1, V@R2C2]
P0 =[0.1 0 0; 0 0.1 0; 0 0 0.1];                                             % Error Co Variance for State Space Equation
Q = 1e-5;                                                                   % Process Noise for State Equation
R = 5e-4;                                                                   % Messaurement Noise for Output Equation
VL(1,1) = 4.2;                                                              % Initial Terminal/Load Voltage
I = 50;                                                                     % We are taking Constant Discharge Current of 50Amps
eff = 0.98;                                                                 % Coulambic Effeciency
Q0 = 50*3600;                                                               % Nominal Capacity of Cell
deltaT = 1;                                                                 % Sampling time , 1 Time per second
N = 3600/deltaT;                                                            % Total Number of Iteration
error(1,1) = 0;                                                             % Initial Error Value

for t = 2:N
    SOC = X(1,t-1);                                                         % Initaial SoC 
    R_int = 0.00573 - 0.03427*SOC^1 + 0.1455*SOC^2 - 0.32647*SOC^3 ...
            + 0.41465*SOC^4 - 0.28992*SOC^5 + 0.09353*SOC^6 -...
            0.00634*SOC^7;
    R1 = 0.01513 - 0.18008*SOC^1 + 1.05147*SOC^2 - 3.27616*SOC^3 ...
         + 5.79793*SOC^4 - 5.81819*SOC^5 + 3.08032*SOC^6 -...
         0.66827*SOC^7;
    C1 = 47718.90713 - 1.00583E6*SOC^1 + 9.2653E6*SOC^2 -3.91088E7*SOC^3 ...
        + 8.85892E7*SOC^4 - 1.11014E8*SOC^5 + 7.22811E7*SOC^6 -...
        1.90336E7*SOC^7;
    R2 = 0.01513 - 0.18008*SOC^1 + 1.05147*SOC^2 - 3.27616*SOC^3 ...
        + 5.79793*SOC^4 - 5.81819*SOC^5 + 3.08032*SOC^6 -...
        0.66827*SOC^7;
    C2 = 47718.90713 - 1.00583E6*SOC^1 + 9.2653E6*SOC^2 -3.91088E7*SOC^3 ...
        + 8.85892E7*SOC^4 - 1.11014E8*SOC^5 + 7.22811E7*SOC^6 - ...
        1.90336E7*SOC^7;
    
   Uocv = -23.60229*SOC^7 + 141.34077*SOC^6 -314.92282*SOC^5 ...
        + 345.34531*SOC^4 -200.15462*SOC^3 ...
        + 60.21383*SOC^2 -7.88447*SOC+3.2377;
    
    tau1 = R1*C1;
    tau2 = R2*C2;
    
    A1 = 1;
    A2 = exp(-deltaT/tau1);
    A3 = exp(-deltaT/tau2);
    
    A = [A1 0 0; 0 A2 0;0 0 A3];                                             % State Transition Matrix
    
    B1 = -deltaT*eff/Q0;
    B2 = R1*(1-exp(-deltaT/tau1));
    B3 = R2*(1-exp(-deltaT/tau2));
    
    B = [B1;B2;B3];                                                         % Input Co-effecient Matrix
    
    X(:,t) = A*X(:,t-1) + B*I + sqrt(Q)*randn(3,1);
    V_RC1 = X(2,t);
    V_RC2 = X(3,t);
    
   
    % Output Equation
    UL(1,t) = Uocv - V_RC1-V_RC2- I*R_int + sqrt(Q)*randn(1,1);
    
    
end


%% Kalman Filter Implementation
X_update = [0.9;0;0];                                                       % Updated State 
X_predict = [0;0;0];                                                        % Predict state

for t = 2: N
    SOC = X_update(1,t-1);
    
     R_int = 0.00573 - 0.03427*SOC^1 + 0.1455*SOC^2 - 0.32647*SOC^3 ...
            + 0.41465*SOC^4 - 0.28992*SOC^5 + 0.09353*SOC^6 -...
            0.00634*SOC^7;
    R1 = 0.01513 - 0.18008*SOC^1 + 1.05147*SOC^2 - 3.27616*SOC^3 ...
         + 5.79793*SOC^4 - 5.81819*SOC^5 + 3.08032*SOC^6 -...
         0.66827*SOC^7;
    C1 = 47718.90713 - 1.00583E6*SOC^1 + 9.2653E6*SOC^2 -3.91088E7*SOC^3 ...
        + 8.85892E7*SOC^4 - 1.11014E8*SOC^5 + 7.22811E7*SOC^6 -...
        1.90336E7*SOC^7;
    R2 = 0.01513 - 0.18008*SOC^1 + 1.05147*SOC^2 - 3.27616*SOC^3 ...
        + 5.79793*SOC^4 - 5.81819*SOC^5 + 3.08032*SOC^6 -...
        0.66827*SOC^7;
    C2 = 47718.90713 - 1.00583E6*SOC^1 + 9.2653E6*SOC^2 -3.91088E7*SOC^3 ...
        + 8.85892E7*SOC^4 - 1.11014E8*SOC^5 + 7.22811E7*SOC^6 - ...
        1.90336E7*SOC^7;
    
  
    tau1 = R1*C1;
    tau2 = R2*C2;
    
    A1 = 1;
    A2 = exp(-deltaT/tau1);
    A3 = exp(-deltaT/tau2);
    
    % State Transition Matrix
    A = [A1 0 0; 0 A2 0; 0 0 A3];
    
    % Input Co-effecient Matrix
    B1 = -deltaT*eff/Q0;
    B2 = R1*(1-exp(-deltaT/tau1));
    B3 = R2*(1-exp(-deltaT/tau2));
    
    B = [B1; B2; B3;];
    
    % 1. Determining the State
    X_predict(:,t) = A*X_update(:,t-1) + B*I;
    SOC_predict = X_predict(1,t);
    V_RC1_predict = X_predict(2,t);
    V_RC2_predict = X_predict(3,t);
    
    Uocv = -23.60229*SOC_predict^7 + 141.34077*SOC_predict^6 -314.92282*SOC_predict^5 ...
        + 345.34531*SOC_predict^4 -200.15462*SOC_predict^3 ...
        + 60.21383*SOC_predict^2 -7.88447*SOC_predict+3.2377;
    
    VL = Uocv - V_RC1_predict - V_RC2_predict - I*R_int;
    
    % 2. Calculating the Error CoVariance
    
    % P(k+1/k) = A*P(k/k)*A' + R
    
    P1 = A * P0 * A' + [Q 0 0; 0 Q 0; 0 0 Q];
    
    % 3. Calculating the Kalman Gain
    
    Cu=-23.60229*7*SOC_predict^6 + 141.34077*6*SOC_predict^5 -314.92282*5*SOC_predict^4 ...
        + 345.34531*4*SOC_predict^3 -200.15462*3*SOC_predict^2 ...
        + 60.21383*2*SOC_predict -7.88447;
    
    C = [Cu -1 -1];
    
    % K(k+1) = P(k+1/k)*C'/ C * P(k+1/k) * C' + R
    
    K = (P1 * C' )/( C * P1 * C' + R);
    
    % 4. Updated State Estimation/ Corrction on Prediction States
    
    % X(k+1/k+1) = X(k/k+1) + K(k+1)*(Y(k+1) - Yest(k+1))x
    
    X_update(:,t) = X_predict(:,t) + K *(UL(1,t) - VL);
    
    
    % 5. Updated Error Co-Variance 
    % P(k+1/k+1) = [I- K(k+1)*C]* P(k+1/k)
    
    P0 = P1 - K*C*P1;
    
    
    
   
end


%% Visualising the Plot
t = 1 : N;
figure(1)
grid on
hold on

%plot(t, X_update(1,:));
plot(t, X_update(1,:),'.',t, X(1,:), 'g',t,X_predict(1,:), 'r');
h = xlabel('Time $ t $ [s]');
set(h,'Interpreter','Latex');
h = ylabel('State of Charge [%]');
set(h,'Interpreter','Latex');
legend('SoC_Update', 'SoC Without EKF', 'SoC_predict', 'Location','Best');
title("State of Charge");
set(h,'Interpreter', 'latex');
h = gca;
set(h,'FontName', 'Times', 'FontSize', 20);
%print -r300 -djpeg State_of_charge;


