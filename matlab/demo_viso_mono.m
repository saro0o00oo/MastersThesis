% demonstrates mono visual odometry on an image sequence
disp('===========================');
clear all; close all; dbstop error;

%%%%%%%%%%%%%%%%%%%%%%%%%%INPUT%%%%%%%%%%%%%%%%%%%%%%
%need to change
ResultDir='C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\00\';
DataDir='C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\00\';
GPSdir='C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\00.txt';
skipFrame=1;


% parameter settings
img_dir     = [DataDir 'image_0\'];

%Calibration
fileID = fopen([DataDir 'calib2.txt'],'r');
formatSpec = '%f';
calib = fscanf(fileID,formatSpec);


%GPS
fileID =  fopen(GPSdir ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[12,size(A,1)/12]);

param.f     = calib(1);
param.cu    = calib(2);
param.cv    = calib(3);
param.height = 1.65;
param.pitch  = -0.06;
first_frame  = 0;
%last_frame   = size(A,1)/12-2;
last_frame   = 1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%START%%%%%%%%%%%%%%%%%
% init visual odometry
visualOdometryMonoMex('init',param);

% init transformation matrix array
Tr_totalM{1} = eye(4);

% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
set(gca,'XTick',-500:10:500);
set(gca,'YTick',-500:10:500);
axis equal, grid on, hold on;

%load('Frameg1');
% for all frames do
replace = 0;
for frame=first_frame:last_frame
  
  % 1-based index
  k = frame-first_frame+1;
  
  I = imread([img_dir num2str(frame,'%06d') '.png']);

  % compute egomotion
  Tr = visualOdometryMonoMex('process',I,replace);

  if k>1
    
    % if motion estimate failed: set replace "current frame" to "yes"
    % this will cause the "last frame" in the ring buffer unchanged
    if isempty(Tr)
        TrM(:,:,k)=eye(4);
      replace = 1;
      Tr_totalM{k} = Tr_totalM{k-1};
      
    % on success: update total motion (=pose)
    else
        TrM(:,:,k)=Tr;
      replace = 0;
      
      Tr_totalM{k} = Tr_totalM{k-1}*inv(Tr);
    end
  end

  % update image
  axes(ha1); cla;
  imagesc(I); colormap(gray);
  axis off;
  
  % update trajectory
  axes(ha2);
  if k>1
    plot([Tr_totalM{k-1}(1,4) Tr_totalM{k}(1,4)], ...
         [Tr_totalM{k-1}(3,4) Tr_totalM{k}(3,4)],'-xb','LineWidth',1);
  end
  pause(0.05); refresh;

  % output statistics
  num_matches = visualOdometryMonoMex('num_matches');
  num_inliers = visualOdometryMonoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
end



% release visual odometry
visualOdometryMonoMex('close');

save([ ResultDir 'TrM.mat'],'TrM')
save([ResultDir 'Tr_totalM.mat'],'Tr_totalM')