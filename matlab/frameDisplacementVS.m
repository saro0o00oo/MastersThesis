clc
clear all
close all

%program that shows t,r transformation to image frames displacement


% demonstrates stereo visual odometry on an image sequence
disp('===========================');
clear all; close all; dbstop error;

%%%%%%%%%%%%%%%%%%%%%INPUT%%%%%%%%%%%%%%%
% parameter settings
img_dir     = 'C:\Users\alizades\Google Drive\Master\Code\Data\httpwww.cvlibs.netdatasetskarlsruhe_sequences\2009_09_08_drive_0015';

%GPS
fileID = fopen('C:\Users\alizades\Google Drive\Master\Code\Data\httpwww.cvlibs.netdatasetskarlsruhe_sequences\2009_09_08_drive_0015\insdata.txt','r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[10,size(A,1)/10]);
%columns are: timestamp,lat,lon,alt,x,y,z,roll,pitch,yaw.


param.f     = 645.2;
param.cu    = 635.9;
param.cv    = 194.1;
param.base  = 0.571;
first_frame = 0;
%last_frame  = size(A,1)/10-1;
last_frame  = 20;
skip=1;

%%%%%%%%%%%%%%%%%%%%%%%%%START%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% init visual odometry
visualOdometryStereoMex('init',param);

% init transformation matrix array
Tr_total{1} = eye(4);


% for all frames do
for frame=first_frame:skip:last_frame
  
  % 1-index
  k = frame-first_frame+1;
  
  % read current images
  I1 = imread([img_dir '/I1_' num2str(frame,'%06d') '.png']);
  I2 = imread([img_dir '/I2_' num2str(frame,'%06d') '.png']);

%   if frame>0
%   figure;
%   subplot(3,1,1)
%   imshow(Ip);
%   subplot(3,1,2)
%   imshow(I1)
%   subplot(3,1,3)
%   imshow(I1-Ip)
%   
%   end
  
  % compute and accumulate egomotion
  Tr = visualOdometryStereoMex('process',I1,I2);
  if k>1
    Tr_total{k} = Tr_total{k-skip}*inv(Tr);
  end
  

  % output statistics
  num_matches = visualOdometryStereoMex('num_matches');
  num_inliers = visualOdometryStereoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
    
    Ip=I1;
end

% release visual odometry
visualOdometryStereoMex('close');
