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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           parameters            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% bucketing parameters
max_features         = int32(2);
bucket_width         = 100;
bucket_height        = 50;

% odometry/camera parameters
initialState         = [0; 0; -20; 0; 0; 0];
initialCovState      = diag([2, 2, 2, 1, 1, 1].^2);
covSystem            = diag([1, 1, 1, 0.05, 0.05, 0.05].^2); %m/s, rad
varMeasurements      = 1;
intrinsicCalibration = [340 0 330;0 340 94;0 0 1];
extrinsicRotation    = eye(3);
extrinsicTranslation = [-0.575;0;0];
deltaT               = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     create test trajectory      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create random stream with seed
rstream1=RandStream('mrg32k3a','Seed',1); % trajectory
rstream2=RandStream('mrg32k3a','Seed',2); % features

% synthesize transformation matrices for all frames
alpha = 0;
for i = 1:40
  alpha = 0.95*(alpha + 0.05*randn(rstream1));
  Tr_ref{i} = [cos(alpha) 0 sin(alpha) 0;0 1 0 0; -sin(alpha) 0 cos(alpha) -2; 0 0 0 1];
  if i==1
    Tr_ref_total{i} = Tr_ref{i}^-1;
  else
    Tr_ref_total{i} = Tr_ref_total{i-1}*Tr_ref{i}^-1;
  end
end

% projection matrix from left to right camera coordinate system
Tr_left_right = [extrinsicRotation,extrinsicTranslation;0 0 0 1];

% create 3d points and 2d matches for all frames
num_features = 40;
for i=1:length(Tr_ref)
  
  % create 3d reference points
  p3_ref = [];
  for j=1:num_features
    p3_ref = [p3_ref; randn(rstream2)*10 randn(rstream2)*5 15+rand(rstream2)*20];
  end
  
  % 3d points in camera coordinate systems
  p3{i}.lp = p3_ref;
  p3{i}.rp = project(p3_ref,Tr_left_right);
  p3{i}.lc = project(p3_ref,Tr_ref{i});
  p3{i}.rc = project(p3_ref,Tr_left_right*Tr_ref{i});
  
  % project to image planes
  p2{i}.lp = project(p3{i}.lp,intrinsicCalibration)+randn(rstream2,size(p3{i}.lp,1),2)*0;%sqrt(varMeasurements);
  p2{i}.rp = project(p3{i}.rp,intrinsicCalibration)+randn(rstream2,size(p3{i}.rp,1),2)*0;%sqrt(varMeasurements);
  p2{i}.lc = project(p3{i}.lc,intrinsicCalibration)+randn(rstream2,size(p3{i}.lc,1),2)*0;%sqrt(varMeasurements);
  p2{i}.rc = project(p3{i}.rc,intrinsicCalibration)+randn(rstream2,size(p3{i}.rc,1),2)*0;%sqrt(varMeasurements);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     compute visual odometry     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% init matcher + odometry objects
matcherMex('init');
visualOdometryMex('init',initialState,initialCovState,covSystem,...
                         intrinsicCalibration,extrinsicRotation,extrinsicTranslation);
                       
% create figure
figure; axis equal; hold on;

% for all frames do
for i=1:length(Tr_ref)
  
  % output
  disp(['Frame: ' num2str(i)]);

  % estimate egomotion
  p_matched = single([p2{i}.lp p2{i}.rp p2{i}.lc p2{i}.rc]');
  visualOdometryMex('update',deltaT,p_matched,varMeasurements);

  % get state vector and inliers
  Tr        = visualOdometryMex('gettransformation');
  inliers   = visualOdometryMex('getinliers');
  Tr_odo{i} = [Tr;0 0 0 1];
  
  % accumulate total motion
  if i==1
    Tr_odo_total{i} = Tr_odo{i}^-1;
  else
    Tr_odo_total{i} = Tr_odo_total{i-1}*Tr_odo{i}^-1;
    
    % show resulting trajectory
    plot([Tr_ref_total{i-1}(1,4) Tr_ref_total{i}(1,4)], ...
         [Tr_ref_total{i-1}(3,4) Tr_ref_total{i}(3,4)],'-xr','LineWidth',1);
    plot([Tr_odo_total{i-1}(1,4) Tr_odo_total{i}(1,4)], ...
         [Tr_odo_total{i-1}(3,4) Tr_odo_total{i}(3,4)],'-xb','LineWidth',1);
       
    % redraw
    pause(0.01);
    refresh;
  end
end

% release objects
matcherMex('close');
visualOdometryMex('close');
