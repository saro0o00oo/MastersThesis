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

clear variables; close all; clc;
rand('state',0); % rand('state',sum(100*clock));

%-----------------------------------------------------------------------
% Sequence, camera and filter tuning parameters, variable initialization
%-----------------------------------------------------------------------

% Camera calibration
cam = initialize_cam;

% Set plot windows
set_plots;

% Sequence path and initial image
sequencePath = '../sequences/ic/rawoutput';
%sequencePath ='C:/Users/alizades/Google Drive/Master/Code/libviso2/matlab/2010_03_09_drive_0019/I1_';
initIm = 60;
lastIm = 180;

% Initialize state vector and covariance
[x_k_k, p_k_k] = initialize_x_and_p;

% Initialize EKF filter
sigma_a = 0.007; % standar deviation for linear acceleration noise
sigma_alpha = 0.007; % standar deviation for angular acceleration noise
sigma_image_noise = 1.0; % standar deviation for measurement noise
filter = ekf_filter( x_k_k, p_k_k, sigma_a, sigma_alpha, sigma_image_noise, 'constant_velocity' );

% variables initialization
features_info = [];
trajectory = zeros( 7, lastIm - initIm );
% other
min_number_of_features_in_image = 25;
generate_random_6D_sphere;
measurements = []; predicted_measurements = [];

%---------------------------------------------------------------
% Main loop
%---------------------------------------------------------------

im = takeImage( sequencePath, initIm );
X_k_k={};
X_k_km1={};
P_k_k={};
P_k_km1={};
ii=1;

pose1=eye(4);
pose2=eye(4);

for step=initIm+1:lastIm
    
    figure(2);
    imshow(im);
    
    
    % Map management (adding and deleting features; and converting inverse depth to Euclidean)
    [ filter, features_info ] = map_management( filter, features_info, cam, im, min_number_of_features_in_image, step );
    
    for i=1:size(features_info,2)
    c(i,:)=features_info(i).uv_when_initialized;
    end
%     figure(3);
% imshow(im)
% axis image
% %colormap(gray)
% hold on
% %plot(cs(:,1), cs(:,2), 'r.')
% plot(c(:,1), c(:,2), 'g.')
% legend('nonmax-suppressed corners')
% title('9 point FAST corner detection on an image')


    x_k_k=get_x_k_k(filter);
    p_k_k=get_p_k_k(filter);
    x_k_km1=get_x_k_km1(filter);
    p_k_km1=get_p_k_km1(filter);

    % EKF prediction (state and measurement prediction)
    [ filter, features_info ] = ekf_prediction( filter, features_info );
    
    % Grab image
    im = takeImage( sequencePath, step );
    Im(:,:,ii)=im;
    
    
    x_k_k=get_x_k_k(filter);
    p_k_k=get_p_k_k(filter);
    x_k_km1=get_x_k_km1(filter);
    p_k_km1=get_p_k_km1(filter);
    
    
    % Search for individually compatible matches
    features_info = search_IC_matches( filter, features_info, cam, im );
    
    % 1-Point RANSAC hypothesis and selection of low-innovation inliers
    features_info = ransac_hypotheses( filter, features_info, cam );
    
    % Partial update using low-innovation inliers
    filter = ekf_update_li_inliers( filter, features_info );
    
    % "Rescue" high-innovation inliers
    features_info = rescue_hi_inliers( filter, features_info, cam );
    
    % Partial update using high-innovation inliers
    filter = ekf_update_hi_inliers( filter, features_info );
    
    
    X_k_k{ii}=x_k_k;
    X_k_km1{ii}=x_k_km1;
    
%     x_k_k=get_x_k_k(filter);
%     T1(:,ii)=x_k_k(1:13);
%     R1ii(:,:,ii)=q2r(T1(4:7,ii));
%     Tr1(:,:,ii)=[R1ii(1,1,ii),R1ii(1,2,ii),R1ii(1,3,ii),T1(1,ii);...
%         R1ii(2,1,ii),R1ii(2,2,ii),R1ii(2,3,ii),T1(2,ii);...
%         R1ii(3,1,ii),R1ii(3,2,ii),R1ii(3,3,ii),T1(3,ii);0,0,0,1];
%     pose1=pose1*inv(Tr1(:,:,ii));
%     Pose1(:,:,ii)=pose1;
%     X_k_k{ii}=x_k_k;
%     
%     
%     x_k_km1=get_x_k_km1(filter);
%     T2(:,ii)=x_k_km1(1:13);
%     R2ii(:,:,ii)=q2r(T2(4:7,ii));
%     Tr2(:,:,ii)=[R2ii(1,1,ii),R2ii(1,2,ii),R2ii(1,3,ii),T2(1,ii);...
%         R2ii(2,1,ii),R2ii(2,2,ii),R2ii(2,3,ii),T2(2,ii);...
%         R2ii(3,1,ii),R2ii(3,2,ii),R2ii(3,3,ii),T2(3,ii);0,0,0,1];
%     pose2=pose2*inv(Tr2(:,:,ii));
%     Pose2(:,:,ii)=pose2;
%     X_k_km1{ii}=x_k_km1;
%     
%     p_k_k=get_p_k_k(filter);
%     P_k_k{ii}=p_k_k;
%     p_k_km1=get_p_k_km1(filter);
%     P_k_km1{ii}=p_k_km1;
    ii=ii+1;
    % Plots,
    plots; display( i );
    
    % Save images
    % saveas( figure_all, sprintf( '%s/image%04d.fig', directory_storage_name, step ), 'fig' );
    
end

% Mount a video from saved Matlab figures
% fig2avi;