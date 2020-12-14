% demonstrates stereo visual odometry on an image sequence
disp('===========================');
clear all; close all; dbstop error;

%%%%%%%%%%%%%%%%%%%%%INPUT%%%%%%%%%%%%%%%
% parameter settings
% img_dir     = 'C:\Users\alizades\Google Drive\Master\Code\Data\httpwww.cvlibs.netdatasetskarlsruhe_sequences\2009_09_08_drive_0015';
% 
% %GPS
% fileID = fopen('C:\Users\alizades\Google Drive\Master\Code\Data\httpwww.cvlibs.netdatasetskarlsruhe_sequences\2009_09_08_drive_0015\insdata.txt','r');
% formatSpec = '%f';
% A = fscanf(fileID,formatSpec);
% Tr_total_GPS=reshape(A,[10,size(A,1)/10]);
% %columns are: timestamp,lat,lon,alt,x,y,z,roll,pitch,yaw.
% 
% 
% param.f     = 645.2;
% param.cu    = 635.9;
% param.cv    = 194.1;
% param.base  = 0.571;
% first_frame = 0;
% last_frame  = size(A,1)/10-1;
% %last_frame  = 500;
% 
% 

%need to change
sequence='08';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

skipFrame=1;


% parameter settings
img_dir1     = [DataDir 'image_0\'];
img_dir2     = [DataDir 'image_1\'];

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
param.height = 1.7;
param.pitch  = -0.03;
first_frame  = 0;
%last_frame=1060;%seq 12
%last_frame=3280;%seq 13
%last_frame=630;%seq 14
%last_frame=1900;%seq 15
last_frame   = size(A,1)/12-2;
%last_frame   = 200;



%%%%%%%%%%%%%%%%%%%%%%%%%START%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init visual odometry
visualOdometryStereoMex('init',param);

% init transformation matrix array
Tr_total{1} = eye(4);

% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.07,0.79,0.68,0.2]);

ha3= axes('Position',[0.75,0.55,0.2,0.2]);

ha4= axes('Position',[0.75,0.7,0.2,0.2]);
axes(ha4);
title('Speed(Km/h)','FontWeight','bold','FontSize',13,'Color','r')

axis off;
ha2 = axes('Position',[0.1,0.1,0.7,0.6]);
set(gca,'XTick',-500:10:500);
set(gca,'YTick',-500:10:500);
axis equal, grid on, hold on;
xlabel('x(m)','FontWeight','bold');


% xlabh = get(gca,'XLabel');
% set(xlabh,'Position',get(xlabh,'Position') + [0 .5 0]);
ylabel('z(m)','FontWeight','bold');

writerObj = VideoWriter('out.avi','Uncompressed AVI'); % Name it.
writerObj.FrameRate = 20; % How many frames per second.
open(writerObj);


% for all frames do
for frame=first_frame:last_frame
    
    % 1-index
    k = frame-first_frame+1;
    
    % read current images
    I1 = imread([img_dir1 num2str(frame,'%06d') '.png']);
    I2 = imread([img_dir2 num2str(frame,'%06d') '.png']);
    
    % compute and accumulate egomotion
    Tr = visualOdometryStereoMex('process',I1,I2);
    T_r(:,:,k)=Tr;
    Dasl(frame+1)=sqrt(Tr(1,4)^2+Tr(3,4)^2);
    Vasl(frame+1)=Dasl(frame+1)*10*3600/1000;
    if k>1
        Tr_total{k} = Tr_total{k-1}*inv(Tr);
    end
    
    % update image
    axes(ha1); cla;
    imagesc(I1); colormap(gray);
    axis off;
    
    axes(ha3);
    title([num2str(Vasl(frame+1))],'FontWeight','bold','FontSize',20,'Color','r')
    axis off;
    
    % update trajectory
    axes(ha2);
    if k>1
        plot([Tr_total{k-1}(1,4) Tr_total{k}(1,4)], ...
            [Tr_total{k-1}(3,4) Tr_total{k}(3,4)],'xr','LineWidth',1);
    end
    pause(0.05); refresh;
    
    frm = getframe(gcf);
    %%saveas(gcf,['r' num2str(frame,'%d')],'jpg');
    writeVideo(writerObj, frm);
    
    % output statistics
    num_matches = visualOdometryStereoMex('num_matches');
    num_inliers = visualOdometryStereoMex('num_inliers');
    disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
    
    
end
close(writerObj); % Saves the movie.

% release visual odometry
visualOdometryStereoMex('close');
