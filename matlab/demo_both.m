% Copyright 2010. All rights reserved.
% Institute of Measurement and Control Systems
% Karlsruhe Institute of Technology, Germany

% This file is part of libviso.
% Authors: Bernd Kitt, Andreas Geiger

% libviso is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 2 of the License, or any later version.

% libviso is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% libviso; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA 

clear all; close all; dbstop error;

% bucketing parameters
max_features         = int32(2);
bucket_width         = 100;
bucket_height        = 50;

% odometry parameters
initialState         = [0; 0; 0; 0; 0; 0];
initialCovState      = diag([2, 2, 2, 1, 1, 1].^2);
covSystem            = diag([1, 1, 1, 0.05, 0.05, 0.05].^2); %m/s, rad
varMeasurements      = 1;
intrinsicCalibration = [340 0 330;0 340 94;0 0 1];
extrinsicRotation    = eye(3);
extrinsicTranslation = [-0.575;0;0];
deltaT               = 0.1;

% init matcher + odometry objects
matcherMex('init');
visualOdometryMex('init',initialState,initialCovState,covSystem,...
                         intrinsicCalibration,extrinsicRotation,extrinsicTranslation);

% init transformation matrix array
Tr_total{1} = eye(4);

% create figure
figure; subplot(2,1,2); axis equal; hold on;

% for all frames do
for frame=1:200
  
  % read current images
  I1 = int16(imread(['../img/' num2str(frame,'%06d')  '_left.jpg']));
  I2 = int16(imread(['../img/' num2str(frame,'%06d') '_right.jpg']));
  matcherMex('push',I1',I2');

  % start matching after reading 2nd frame  
  if frame>1

    % match features
    p_matched = matcherMex('match',max_features,bucket_width,bucket_height);
    disp(['Frame: ' num2str(frame), ' Matches after bucketing: ' num2str(size(p_matched,2))]);
        
    % estimate egomotion
    visualOdometryMex('update',deltaT,p_matched,varMeasurements);

    % get transformation matrix and inliers
    Tr      = visualOdometryMex('gettransformation');
    inliers = visualOdometryMex('getinliers');
    
    % compute transformation matrix and accumulate total transformation
    Tr              = [Tr;0 0 0 1];
    Tr_total{frame} = Tr_total{frame-1}*Tr^-1;
    
    % show current image with inlier (color) and outlier matches (blue)
    subplot(2,1,1); cla;
    plotMatch(I1,p_matched,inliers);
    
    % draw trajectory
    subplot(2,1,2);
    plot([Tr_total{frame-1}(1,4) Tr_total{frame}(1,4)], ...
         [Tr_total{frame-1}(3,4) Tr_total{frame}(3,4)],'-xr','LineWidth',1);
    
    % display
    pause(0.01);
    refresh;
  end
end

% release objects
matcherMex('close');
visualOdometryMex('close');
