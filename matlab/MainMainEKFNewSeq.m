clc
clear all
close all


%need to change
sequence='11';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];


 load([ResultDir 'TrM']);
 load([ResultDir 'Tr_totalM']);
% load([ResultDir 'Tr']);
% load([ResultDir 'Tr_total']);

%GPS
fileID = fopen(GPSdir ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
XZ_GPS=reshape(A,[4,size(A,1)/4]);
for i=1:size(XZ_GPS,2)
    X_GPS(i)=XZ_GPS(1,i);
    Z_GPS(i)=XZ_GPS(2,i);
    X_Seq(i)=XZ_GPS(3,i);
    Z_Seq(i)=XZ_GPS(4,i);
end

% %GPS
% fileID = fopen(GPSdir ,'r');
% formatSpec = '%f';
% A = fscanf(fileID,formatSpec);
% Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
% for i=1:size(Tr_total_GPS,2)
%     Tr_total_GPS_P{i}=reshape(Tr_total_GPS(:,i),4,3)';
% end


%% Measurment

for i=1:size(TrM,3)
    tr_t(:,i)=TrM(1:3,4,i);
    ry1=asin(TrM(1,3,i));
    rx1=asin(-TrM(2,3,i)/cos(ry1));
    rz1=asin(-TrM(1,2,i)/cos(ry1));
    tr_r(:,i)=[rx1;ry1;rz1];
end

zr=tr_r;
zt=tr_t;
z=[zt;zr];


for i=1:1:size(z,2)
    if z(1,i)> .1
        z(1,i)=.1;
    end
    if z(1,i)<-.1
        z(1,i)=-.1;
    end
    if z(3,i)> .2
        z(3,i)=.2;
    end
    if z(3,i)<-2
        z(3,i)=-2;
    end
end


%% Parameters

N_k         = size(z,2)-2; %num of frame
%N_k=700;
S_k= 2; %starting frame
S_p=2;

dt=1;

Tr_total_hat{S_p}=eye(4);


%State Eq.
%X = [t_x;t_y;t_z;qw;q_x;q_t;q_z;t_x/dt;t_y/dt;t_z/dt;qw/dt;q_x/dt;q_t/dt;q_z/dt]; %state vector


%Measurment Eq.
%z=[t_x;t_y;t_z;r_x;r_y;r_z];
H=[eye(7,7) zeros(7,6)];
R           =  diag(1e-4*ones(1,7));%Measurement Noise cov. matrix


%Process noise
[ sigma_a,sigma_alpha,z ] = processNoise( '06',z );


%% Extended Kalman Filter


for i=1:size(z,2)
    qz(:,i)=v2q(z(4:6,i));
end

z_Q=[z(1:3,:);qz];


type='constant_velocity';
%type='constant_position';
%type='constant_orientation';
%type='constant_position_and_orientation';
%type='constant_position_and_orientation_location_noise';

xhat  = ExtnddKalmanFilter( N_k,S_k,H,R,z_Q,dt,type,sigma_a,sigma_alpha );

for i=S_k:N_k+1 
    %xhatEuler(:,i)=q2v(xhat(4:7,i));
    xhatR_Euler(:,:,i)=q2tr(xhat(4:7,i));
end


for k=S_p:N_k
    
    %     Tr_hat(:,:,k+1)=[cos(ry_hat(k+1))*cos(rz_hat(k+1)),-cos(ry_hat(k+1))*sin(rz_hat(k+1)),sin(ry_hat(k+1)),xhat(1,k+1)...
    %         ;sin(rx_hat(k+1))*sin(ry_hat(k+1))*cos(rz_hat(k+1))+cos(rx_hat(k+1))*sin(rz_hat(k+1)),-sin(rx_hat(k+1))*sin(ry_hat(k+1))*sin(rz_hat(k+1))+cos(rx_hat(k+1))*cos(rz_hat(k+1)),-sin(rx_hat(k+1))*cos(ry_hat(k+1)),xhat(2,k+1)...
    %         ;-cos(rx_hat(k+1))*sin(ry_hat(k+1))*cos(rz_hat(k+1))+sin(rx_hat(k+1))*sin(rz_hat(k+1)),cos(rx_hat(k+1))*sin(ry_hat(k+1))*sin(rz_hat(k+1))+sin(rx_hat(k+1))*cos(rz_hat(k+1)),cos(rx_hat(k+1))*cos(ry_hat(k+1)),xhat(3,k+1)...
    %         ;0,0,0,1];
    
    Tr_hat(:,:,k+1)=[xhatR_Euler(1:3,1:3,k+1),[xhat(1,k+1);xhat(2,k+1);xhat(3,k+1)];0,0,0,1];
    
    Tr_total_hat{k+1}=Tr_total_hat{k}*inv(Tr_hat(:,:,k+1));
    
end
%% Ploting

for i= S_p:1:N_k
    
    X_hat(i)=Tr_total_hat{i+1}(1,4);
    Z_hat(i)=Tr_total_hat{i+1}(3,4);
    Y_hat(i)=Tr_total_hat{i+1}(2,4);
    
    X(i)=Tr_totalM{i}(1,4)-Tr_totalM{S_p-1}(1,4);
    Z(i)=Tr_totalM{i}(3,4)-Tr_totalM{S_p-1}(3,4);
    Y(i)=Tr_totalM{i}(2,4)-Tr_totalM{S_p-1}(2,4);
    
%     XS(i)=Tr_total{i}(1,4)-Tr_total{S_p-1}(1,4);
%     ZS(i)=Tr_total{i}(3,4)-Tr_total{S_p-1}(3,4);
%     YS(i)=Tr_total{i}(2,4)-Tr_total{S_p-1}(2,4);
    
%     X_gps(i)=Tr_total_GPS_P{i}(1,4)-Tr_total_GPS_P{S_p-1}(1,4);
%     Y_gps(i)=Tr_total_GPS_P{i}(2,4)-Tr_total_GPS_P{S_p-1}(2,4);
%     Z_gps(i)=Tr_total_GPS_P{i}(3,4)-Tr_total_GPS_P{S_p-1}(3,4);
%     
%     if i>1
%         Tr_GPS(:,:,i-1)=inv([Tr_total_GPS_P{i};0 0 0 1])*[Tr_total_GPS_P{i-1};0 0 0 1];
%     end
%     
    
end



for i=S_p:1:N_k-1
    
    %     ry_hat(i)=xhatEuler(2,i);
    %     rx_hat(i)=xhatEuler(1,i);
    %     rz_hat(i)=xhatEuler(3,i);
    
    tx_hat(i)=xhat(1,i);
    ty_hat(i)=xhat(2,i);
    tz_hat(i)=xhat(3,i);
    
    ry_hat(i)=asin(xhatR_Euler(1,3,i));
    rz_hat(i)=abs(acos(xhatR_Euler(1,1,i)/cos(asin(xhatR_Euler(1,3,i)))));
    rx_hat(i)=-asin(xhatR_Euler(2,3,i)/cos(asin(xhatR_Euler(1,3,i))));
    
%     
%     tx_gps(i+1)=Tr_GPS(1,4,i);
%     ty_gps(i+1)=Tr_GPS(2,4,i);
%     tz_gps(i+1)=Tr_GPS(3,4,i);
%     
%     ry_gps(i+1)=asin(Tr_GPS(1,3,i));
%     rz_gps(i+1)=acos(Tr_GPS(1,1,i)/cos(asin(Tr_GPS(1,3,i))));
%     rx_gps(i+1)=-asin(Tr_GPS(2,3,i)/cos(asin(Tr_GPS(1,3,i))));
%     
    
    tx(i)=z(1,i);
    ty(i)=z(2,i);
    tz(i)=z(3,i);
    
    rx(i)=z(4,i);
    ry(i)=z(5,i);
    rz(i)=abs(z(6,i));%mistakeeeeeeee!!!!!!!
end


timeF=[S_p:N_k-1];
figure;
subplot(3,1,1);
plot(timeF,tx(timeF),'b');
hold on
plot(timeF,tx_hat(timeF),'r');
% hold on
% plot(timeF,tx_gps(timeF),'k');
%axis([80 280 -.03 .03])
xlabel('Frame[k]')
ylabel('t_x[m]')
%title('tx');
subplot(3,1,2);
plot(timeF,ty(timeF),'b');
hold on
plot(timeF,ty_hat(timeF),'r');
% hold on
% plot(timeF,ty_gps(timeF),'k');
xlabel('Frame[k]')
ylabel('t_y[m]')
%title('ty');
subplot(3,1,3);
plot(timeF,tz(timeF),'b');
hold on
plot(timeF,tz_hat(timeF),'r');
% hold on
% plot(timeF,tz_gps(timeF),'k');
xlabel('Frame[k]')
ylabel('t_z[m]')
%title('tz');
legend('Viso2M','with KF','GPS')
saveas(gcf,[ResultDir 'tmkf.jpg']);

figure;
subplot(3,1,1);
plot(timeF,rx(timeF),'b');
hold on
plot(timeF,rx_hat(timeF),'r');
xlabel('Frame[k]')
ylabel('r_x[rad]')
% hold on
% plot(timeF,rx_gps(timeF),'k');
%title('rx');
subplot(3,1,2);
plot(timeF,ry(timeF),'b');
hold on
plot(timeF,ry_hat(timeF),'r');
xlabel('Frame[k]')
ylabel('r_y[rad]')
% hold on
% plot(timeF,ry_gps(timeF),'k');
%title('ry');
subplot(3,1,3);
plot(timeF,rz(timeF),'b');
hold on
plot(timeF,rz_hat(timeF),'r');
xlabel('Frame[k]')
ylabel('r_z[rad]')
% hold on
% plot(timeF,rz_gps(timeF),'k');
%title('rz');
legend('Viso2M','with KF','GPS')
saveas(gcf,[ResultDir 'rmkf.jpg']);


figure;
plot(X(timeF),Z(timeF),'b');
hold on
plot(X_hat(timeF),Z_hat(timeF),'r');
% hold on
% plot(X_gps(timeF),Z_gps(timeF),'k')
% hold on
% plot(XS(timeF),ZS(timeF),'g')

hold on
plot(X_Seq,Z_Seq,'r--');
hold on
plot(X_GPS,Z_GPS,'b--');

legend('Viso2M','with KF','Ground truth','Viso2S');
%axis([-50 50 -100 500])
xlabel('x[m]')
ylabel('z[m]')
title('Trajectory')
saveas(gcf,[ResultDir 'DirectionmKF.jpg']);



figure;
subplot(3,1,1);
plot(timeF,X(timeF),'b');
hold on;
plot(timeF,X_hat(timeF),'r');
% hold on;
% plot(timeF,X_gps(timeF),'k');
title('X');
subplot(3,1,2);
plot(timeF,Y(timeF),'b');
hold on
plot(timeF,Y_hat(timeF),'r');
% hold on;
% plot(timeF,Y_gps(timeF),'k');
title('Y');
subplot(3,1,3);
plot(timeF,Z(timeF),'b');
hold on
plot(timeF,Z_hat(timeF),'r');
% hold on;
% plot(timeF,Z_gps(timeF),'k');
title('Z');
legend('VISOM','with EKF','GPS')
saveas(gcf,[ResultDir 'XYZm.jpg']);


%% error metric

% for i=S_k:1:size(Tr_total_hat,2)-1
% E(:,:,i)=(inv(inv(Tr_totalM{i})*Tr_totalM{i+1}))* (inv([Tr_total_GPS_P{i};0,0,0,1])*[Tr_total_GPS_P{i+1};0 0 0 1]);
% E_hat(:,:,i)=(inv(inv(Tr_total_hat{i})*Tr_total_hat{i+1}))* (inv([Tr_total_GPS_P{i};0,0,0,1])*[Tr_total_GPS_P{i+1};0 0 0 1]);
%
% end
%
% for i=S_k:1:size(E,3)
%     RMSE_Et=sqrt(sum(norm(E(1:3,4,i)))/size(E,3));
%     RMSE_Er=sqrt(sum(norm(E(1:3,1:3,i)))/size(E,3));
%     RMSE_Et_hat=sqrt(sum(norm(E_hat(1:3,4,i)))/size(E_hat,3));
%     RMSE_Er_hat=sqrt(sum(norm(E_hat(1:3,1:3,i)))/size(E_hat,3));
% end
