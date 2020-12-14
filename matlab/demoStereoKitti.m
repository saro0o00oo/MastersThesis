% demonstrates stereo visual odometry on an image sequence
disp('===========================');
clear all; close all; dbstop error;

%%%%%%%%%%%%%%%%%%%%%INPUT%%%%%%%%%%%%%%%
% parameter settings

%need to change
sequence='07';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

img_dirL     = [DataDir 'image_0\'];
img_dirR     = [DataDir 'image_1\'];

%Calibration
fileID = fopen([DataDir 'calib2.txt'],'r');
formatSpec = '%f';
calib = fscanf(fileID,formatSpec);

%columns are: timestamp,lat,lon,alt,x,y,z,roll,pitch,yaw.

%GPS
fileID = fopen(GPSdir ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
%columns are: timestamp,lat,lon,alt,x,y,z,roll,pitch,yaw.


param.f     = calib(1);
param.cu    = calib(2);
param.cv    = calib(3);
param.base  = abs(calib(4)/calib(1)); %P1(1,4)/P1(1,1)
first_frame = 0;
last_frame  = size(A,1)/12-2;
%last_frame  = 2000;


%%%%%%%%%%%%%%%%%%%%%%%%%START%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% init visual odometry
visualOdometryStereoMex('init',param);

% init transformation matrix array
Tr_total{1} = eye(4);
Tr_total1{1} = eye(4);

ii=1;
% for all frames do
for frame=first_frame:1:last_frame
  
  % 1-index
  k = frame-first_frame+1;
  
  % read current images
  I1 = imread([img_dirL  num2str(frame,'%06d') '.png']);
  I2 = imread([img_dirR  num2str(frame,'%06d') '.png']);

  % compute and accumulate egomotion
  Tr = visualOdometryStereoMex('process',I1,I2);
  Tr11(:,:,k)=Tr;
  Tr1(:,:,ii)=Tr;
  if k>1 && ii>1
    Tr_total1{k} = Tr_total1{k-1}*inv(Tr);
    Tr_total{ii}=Tr_total{ii-1}*inv(Tr);
  end

  % output statistics
  num_matches = visualOdometryStereoMex('num_matches');
  num_inliers = visualOdometryStereoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
    
    ii=ii+1;
end

% release visual odometry
visualOdometryStereoMex('close');

save([ ResultDir 'Tr.mat'],'Tr1')
save([ResultDir 'Tr_total.mat'],'Tr_total')