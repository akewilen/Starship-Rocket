function [x_updated, P_updated, x_pred, P_pred] = EKF(w, a, x, P, Rw ,Ra, T,  f)
% Extended Kalman filter for a quaternion motion model
% F: Linearized quaternion motion model
% H: Linearized accelerometer measurement model
% u: Gyroscope data used as input
% x: Quaternion state
% P: Prior covariance matrix
  

    %Prediction ------------
    
  F = @(w, T) T/2 * Somega(w) + eye(4); % Linearized quaternion motion model
  G = @(x)  T/2 * Sq(x); %Noise part of linearized motion model
    
  x_pred = F(w, T)*x; %prediction step
  x_pred = mu_normalizeQ(x_pred); %Quaternion normalize
  P_pred = F(w, T)*P*F(w, T).' + G(x_pred)*Rw*G(x_pred).';  %P prediction update

 %EKF update step--------------------------------
 if nargin < 8
     f = [0; 0; 0] ; % Force acting on CM resulting in acceleration
 end

 g= [0.0131; -0.3506 ;9.8356]; %Mean of accelerometer data when IMU stationary
 

 H = Acc_measurment_model(x_pred, g, f); %Linearized accelerometer measurement model


 S = H*P_pred*H.' + Ra; %Measurement model covariance update
 K = (P_pred*H.') * pinv(S);

    
 pred_rotation = Q_to_rotation(x_pred).'; 
 yacc_hat = pred_rotation * (g+f);  %nonlinear measurement model
  
 innovation = a - yacc_hat; %Difference between the motion model approximation and the measurement
 x_updated = x_pred + K*innovation;  %State update posterior
 P_updated = P_pred-K*S*K.'; % P posterior update

 x_updated = mu_normalizeQ(x_updated);
   

end