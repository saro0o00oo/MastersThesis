%-----------------------------------------------------------------------
% 1-point RANSAC EKF SLAM from a monocular sequence
%-----------------------------------------------------------------------

% Copyright (C) 2010 Javier Civera and J. M. M. Montiel
% Universidad de Zaragoza, Zaragoza, Spain.

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation. Read http://www.gnu.org/copyleft/gpl.html for details

% If you use this code for academic work, please reference:
%   Javier Civera, Oscar G. Grasa, Andrew J. Davison, J. M. M. Montiel,
%   1-Point RANSAC for EKF Filtering: Application to Real-Time Structure from Motion and Visual Odometry,
%   to appear in Journal of Field Robotics, October 2010.

%-----------------------------------------------------------------------
% Authors:  Javier Civera -- jcivera@unizar.es 
%           J. M. M. Montiel -- josemari@unizar.es

% Robotics, Perception and Real Time Group
% Aragón Institute of Engineering Research (I3A)
% Universidad de Zaragoza, 50018, Zaragoza, Spain
% Date   :  May 2010
%-----------------------------------------------------------------------

function [ X_km1_k, P_km1_k ] = predict_state_and_covariance( X_k, P_k, type, SD_A_component_filter, SD_alpha_component_filter,delta_t )



% camera motion prediction
X_km1_k = fv( X_k, delta_t, type);



% state transition equation derivatives
F = sparse( dfv_by_dxv( X_k,zeros(6,1),delta_t, type ) );

% state noise
linear_acceleration_noise_covariance_x = (SD_A_component_filter(1)*delta_t)^2;
linear_acceleration_noise_covariance_y = (SD_A_component_filter(2)*delta_t)^2;
linear_acceleration_noise_covariance_z = (SD_A_component_filter(3)*delta_t)^2;
angular_acceleration_noise_covariance_x = (SD_alpha_component_filter(1)*delta_t)^2;
angular_acceleration_noise_covariance_y = (SD_alpha_component_filter(2)*delta_t)^2;
angular_acceleration_noise_covariance_z = (SD_alpha_component_filter(3)*delta_t)^2;

Pn = sparse (diag( [linear_acceleration_noise_covariance_x linear_acceleration_noise_covariance_y linear_acceleration_noise_covariance_z...
        angular_acceleration_noise_covariance_x angular_acceleration_noise_covariance_y angular_acceleration_noise_covariance_z] ) );

Q = func_Q( X_k, zeros(6,1), Pn, delta_t, type);

size_P_k = size(P_k,1);

P_km1_k =  F*P_k*F' + Q ;