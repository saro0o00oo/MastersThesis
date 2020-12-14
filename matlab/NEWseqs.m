clc
clear all
%close all


S_p=2;
%need to change
sequence='15';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

load([ResultDir 'Tr_totalM']);
load([ResultDir 'Tr_total_hat']);

fileID = fopen([ResultDir 'Direction-MLMSFM.txt'] ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
A1=reshape(A,[4,size(A,1)/4]);
for i=1:size(A1,2)
    P_MLMSFM(:,i)=A1(3:4,i);
    P_GPS(:,i)=A1(1:2,i);
end

fileID = fopen([ResultDir 'Direction-VISOMGP.txt'] ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
A1=reshape(A,[4,size(A,1)/4]);
for i=1:size(A1,2)
    P_VISOMGP(:,i)=A1(3:4,i);
    P_GPS(:,i)=A1(1:2,i);
end

fileID = fopen([ResultDir 'Direction-VISOM.txt'] ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
A1=reshape(A,[4,size(A,1)/4]);
for i=1:size(A1,2)
    P_VISOM(:,i)=A1(3:4,i);
    P_GPS(:,i)=A1(1:2,i);
end


ii=S_p;
for i= S_p:3:size(Tr_total_hat,2)-1
    
    X(ii)=Tr_totalM{i}(1,4)-Tr_totalM{S_p-1}(1,4);
    Z(ii)=Tr_totalM{i}(3,4)-Tr_totalM{S_p-1}(3,4);  
    X_hat(ii)=Tr_total_hat{i}(1,4);
    Z_hat(ii)=Tr_total_hat{i}(3,4);
    ii=ii+1;
end


for i= S_p:1:size(P_MLMSFM,2)-1
    
    X_MLMSFM(i)=P_MLMSFM(1,i)-P_MLMSFM(1,S_p-1);
    Y_MLMSFM(i)=P_MLMSFM(2,i)-P_MLMSFM(2,S_p-1);
    
    X_VISOMGP(i)=P_VISOMGP(1,i)-P_VISOMGP(1,S_p-1);
    Y_VISOMGP(i)=P_VISOMGP(2,i)-P_VISOMGP(2,S_p-1);
    
    X_VISOM(i)=P_VISOM(1,i)-P_VISOM(1,S_p-1);
    Y_VISOM(i)=P_VISOM(2,i)-P_VISOM(2,S_p-1);
end


figure;
%plot(X_MLMSFM,Y_MLMSFM,'b');
hold on
plot(X_VISOMGP,Y_VISOMGP,'g');
% hold on
% plot(X_VISOM,Y_VISOM,'r');
hold on
plot(X,Z,'b');
hold on
plot(X_hat,Z_hat,'r');
hold on
plot(P_GPS(1,:),P_GPS(2,:),'k');

legend('VISOM-GP','VISOM without KF','VISOM with our filter','Ground truth');
xlabel('x[m]')
ylabel('z[m]')
title('Trajectory')
%saveas(gcf,[ResultDir 'DirectionmKF.jpg']);



