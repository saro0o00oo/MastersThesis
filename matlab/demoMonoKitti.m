% demonstrates mono visual odometry on an image sequence
disp('===========================');
clc; clear all; close all; dbstop error;

%%%%%%%%%%%%%%%%%%%%%%%%%%INPUT%%%%%%%%%%%%%%%%%%%%%%

%need to change
sequence='08';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

skipFrame=1;


% parameter settings
img_dir     = [DataDir 'image_0\'];

%Calibration
fileID = fopen([DataDir 'calib2.txt'],'r');
formatSpec = '%f';
calib = fscanf(fileID,formatSpec);


%GPS
% fileID =  fopen(GPSdir ,'r');
% formatSpec = '%f';
% A = fscanf(fileID,formatSpec);
% Tr_total_GPS=reshape(A,[12,size(A,1)/12]);

param.f     = calib(1);
param.cu    = calib(2);
param.cv    = calib(3);
param.height = 1.7;
param.pitch  = -0.03;
first_frame  = 0;
%last_frame=1060;%seq 12
%last_frame=3280;%seq 13
%last_frame=630;%seq 14
last_frame=1900;%seq 15
%last_frame   = size(A,1)/12-2;
%last_frame   = 200;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%START%%%%%%%%%%%%%%%%%
% init visual odometry
visualOdometryMonoMex('init',param);

% init transformation matrix array
Tr_totalM{1} = eye(4);
Tr_totalM1{1} = eye(4);



ii=1;
% for all frames do
replace = 0;
for frame=first_frame:skipFrame:last_frame
  
  % 1-based index
  k = frame-first_frame+1;
  
  I = imread([img_dir num2str(frame,'%06d') '.png']);

  % compute egomotion
  Tr = visualOdometryMonoMex('process',I,replace);
 
  
  if k>1 && ii>1
    
    % if motion estimate failed: set replace "current frame" to "yes"
    % this will cause the "last frame" in the ring buffer unchanged
    if isempty(Tr)
        TrM1(:,:,k)=eye(4);
        TrM(:,:,ii)=eye(4);
      replace = 1;
      Tr_totalM1{k} = Tr_totalM1{k-skipFrame};
      Tr_totalM{ii}=Tr_totalM{ii-1};
      
    % on success: update total motion (=pose)
    else
        TrM1(:,:,k)=Tr;
        TrM(:,:,ii)=Tr;
      replace = 0;
      Tr_totalM1{k} = Tr_totalM1{k-skipFrame}*inv(Tr);
      Tr_totalM{ii}=Tr_totalM{ii-1}*inv(Tr);
    end
    
  end

  % output statistics
  num_matches = visualOdometryMonoMex('num_matches');
  num_inliers = visualOdometryMonoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
    
    ii=ii+1;
end

% release visual odometry
visualOdometryMonoMex('close');

for i=1:size(Tr_totalM,2)
    Tr_total_Reshape(:,i)=reshape(Tr_totalM{i}(1:3,:),12,1);
end
save([ ResultDir 'TrM.mat'],'TrM')
save([ResultDir 'Tr_totalM.mat'],'Tr_totalM')

fileID = fopen([ResultDir 'Tr_totalM.txt'],'w');
formatSpec = '%f';
fprintf(fileID,formatSpec,Tr_total_Reshape);
fclose(fileID);

fileID = fopen([ResultDir 'Tr_totalM.txt'] ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_totalRR=reshape(A,[12,size(A,1)/12]);
ee= sum(sum(Tr_totalRR-Tr_total_Reshape));