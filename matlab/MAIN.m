clc
clear all
close all


%need to change
ResultDir='C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\07\';
DataDir='C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\07\';
GPSdir='C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\07.txt';
skipFrame=1;%for GPS implementation, it should be the same as demo skipFrame
skipFrame1=1;%for data, keep it 1


load([ResultDir 'TrM']);
load([ResultDir 'Tr_totalM']);

%GPS
fileID = fopen(GPSdir ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
for i=1:size(Tr_total_GPS,2)
Tr_total_GPS_P{i}=reshape(Tr_total_GPS(:,i),4,3)';
end


%% Measurment

for i=1:size(TrM,3)
    tr_t(:,i)=TrM(1:3,4,i);
    ry=asin(TrM(1,3,i));
    rx=asin(-TrM(2,3,i)/cos(ry));
    rz=asin(-TrM(1,2,i)/cos(ry));
    tr_r(:,i)=[rx;ry;rz];
end

z=[tr_r;tr_t];
%z=z(:,100:end);
%  figure
%  plot(1:size(tr_t,2),tr_r(2,:));

%% Parameters

N_k         = size(z,2)-10; %num of frame
%N_k=250;
S_k= 2; %starting frame


dt=1;
x_0 = [0;0;0;0;0;0;0;0;0;0;0;0];%initial true value
P(:,:,1)    = diag(ones(12,1));
P_0=P(:,:,1);
Tr_total_hat{S_k}=eye(4);


%State Eq.
%Constant Velocity
%X = [r_x/dt;r_y/dt;r_z/dt;t_x/dt;t_y/dt;t_z/dt;r_x/dt^2;r_y/dt^2;r_z/dt^2;t_x/dt^2;t_y/dt^2;t_z/dt^2]; %state vector
%ns          = 12;%num of states
%x=ones(12,1);
F           = [eye(6,6) dt*eye(6,6);zeros(6,6) eye(6,6)]; %system matrix
%q           = [mvnrnd(zeros(6,1),10^-6);mvnrnd(zeros(6,1),1)]; %Process Noise var.
Q           = diag([1e-6*ones(1,6) 1e-12*ones(1,6)]);%diag(q);%Process Noise cov. matrix


%Measurment Eq.
%t_r matrix
H=[eye(6,6) zeros(6,6)];
%z=[r_x;r_y;r_z;t_x;t_y;t_z]/dt;
%nm          = 6;%num of measurments
%r           = mvnrnd(zeros(6,1),10^-2); %Measurement Noise var.
R           =  diag([1e-4*ones(1,6)]);%Measurement Noise cov. matrix

%% Kalman Filter

xhat  = KalmanFilter( N_k,S_k,x_0,P_0,F,H,Q,R,z );

for k=S_k:N_k
    
    Tr_hat(:,:,k+1)=[cos(xhat(2,k+1))*cos(xhat(3,k+1)),-cos(xhat(2,k+1))*sin(xhat(3,k+1)),sin(xhat(2,k+1)),xhat(4,k+1)...
        ;sin(xhat(1,k+1))*sin(xhat(2,k+1))*cos(xhat(3,k+1))+cos(xhat(1,k+1))*sin(xhat(3,k+1)),-sin(xhat(1,k+1))*sin(xhat(2,k+1))*sin(xhat(3,k+1))+cos(xhat(1,k+1))*cos(xhat(3,k+1)),-sin(xhat(1,k+1))*cos(xhat(2,k+1)),xhat(5,k+1)...
        ;-cos(xhat(1,k+1))*sin(xhat(2,k+1))*cos(xhat(3,k+1))+sin(xhat(1,k+1))*sin(xhat(3,k+1)),cos(xhat(1,k+1))*sin(xhat(2,k+1))*sin(xhat(3,k+1))+sin(xhat(1,k+1))*cos(xhat(3,k+1)),cos(xhat(1,k+1))*cos(xhat(2,k+1)),xhat(6,k+1)...
        ;0,0,0,1];
    
    Tr_total_hat{k+1}=Tr_total_hat{k}*inv(Tr_hat(:,:,k+1));
    
end
%% Ploting

for i= S_k:1:N_k
   
    X_hat(i)=Tr_total_hat{i}(1,4);
    Z_hat(i)=Tr_total_hat{i}(3,4);
    Y_hat(i)=Tr_total_hat{i}(2,4);
    
    X(i)=Tr_totalM{i}(1,4)-Tr_totalM{S_k-1}(1,4);
    Z(i)=Tr_totalM{i}(3,4)-Tr_totalM{S_k-1}(3,4);
    Y(i)=Tr_totalM{i}(2,4)-Tr_totalM{S_k-1}(2,4);
    
    X_gps(i)=Tr_total_GPS_P{i}(1,4)-Tr_total_GPS_P{S_k-1}(1,4);
    Y_gps(i)=Tr_total_GPS_P{i}(2,4)-Tr_total_GPS_P{S_k-1}(2,4);
    Z_gps(i)=Tr_total_GPS_P{i}(3,4)-Tr_total_GPS_P{S_k-1}(3,4);
    
    if i>1
    Tr_GPS(:,:,i-1)=inv([Tr_total_GPS_P{i};0 0 0 1])*[Tr_total_GPS_P{i-1};0 0 0 1];
    end
    
    
end

for i=S_k:1:N_k-1
    tx_gps(i)=Tr_GPS(1,4,i);
    ty_gps(i)=Tr_GPS(2,4,i);
    tz_gps(i)=Tr_GPS(3,4,i);
    
    ry_gps(i)=asin(Tr_GPS(1,3,i));
    rz_gps(i)=acos(Tr_GPS(1,1,i)/cos(asin(Tr_GPS(1,3,i))));
    rx_gps(i)=-asin(Tr_GPS(2,3,i)/cos(asin(Tr_GPS(1,3,i))));
    
        tx_hat(i)=xhat(4,i);
    ty_hat(i)=xhat(5,i);
    tz_hat(i)=xhat(6,i);
    rx_hat(i)=xhat(1,i);
    ry_hat(i)=xhat(2,i);
    rz_hat(i)=xhat(3,i);
    
    tx(i)=TrM(1,4,i);
    ty(i)=TrM(2,4,i);
    tz(i)=TrM(3,4,i);
    ry(i)=asin(TrM(1,3,i));
    rz(i)=acos(TrM(1,1,i)/cos(asin(TrM(1,3,i))));
    rx(i)=-asin(TrM(2,3,i)/cos(asin(TrM(1,3,i))));
end 
    
figure;
subplot(3,1,1);
plot(S_k:N_k-1,tx(S_k:N_k-1),'b');
hold on
plot(S_k:N_k-1,tx_hat(S_k:N_k-1),'r');
% hold on
% plot(S_k:N_k-1,tx_gps(S_k:N_k-1),'k--');
xlabel('Frame[k]')
ylabel('t_x[m]')
%title('tx');
subplot(3,1,2);
plot(S_k:N_k-1,ty(S_k:N_k-1),'b');
hold on
plot(S_k:N_k-1,ty_hat(S_k:N_k-1),'r');
%  hold on
% plot(S_k:N_k-1,ty_gps(S_k:N_k-1),'k--');
xlabel('Frame[k]')
ylabel('t_y[m]')
%title('ty');
subplot(3,1,3);
plot(S_k:N_k-1,tz(S_k:N_k-1),'b');
hold on
plot(S_k:N_k-1,tz_hat(S_k:N_k-1),'r');
% hold on
% plot(S_k:N_k-1,tz_gps(S_k:N_k-1),'k--');
xlabel('Frame[k]')
ylabel('t_z[m]')
%title('tz');
legend('Viso2M','with KF','GPS')
saveas(gcf,[ResultDir 'tmkf.jpg']);

figure;
subplot(3,1,1);
plot(S_k:N_k-1,rx(S_k:N_k-1),'b');
hold on
plot(S_k:N_k-1,rx_hat(S_k:N_k-1),'r');
xlabel('Frame[k]')
ylabel('r_x[rad]')
% hold on
% plot(S_k:N_k-1,tx_gps(S_k:N_k-1),'k--');
%title('rx');
subplot(3,1,2);
plot(S_k:N_k-1,ry(S_k:N_k-1),'b');
hold on
plot(S_k:N_k-1,ry_hat(S_k:N_k-1),'r');
xlabel('Frame[k]')
ylabel('r_y[rad]')
% hold on
% plot(S_k:N_k-1,ty_gps(S_k:N_k-1),'k--');
%title('ry');
subplot(3,1,3);
plot(S_k:N_k-1,rz(S_k:N_k-1),'b');
 hold on
 plot(S_k:N_k-1,rz_hat(S_k:N_k-1),'r');
 xlabel('Frame[k]')
ylabel('r_z[rad]')
% hold on
% plot(S_k:N_k-1,tz_gps(S_k:N_k-1),'k--');
%title('rz');
legend('Viso2M','with KF','GPS')
saveas(gcf,[ResultDir 'rmkf.jpg']);


figure;
plot(X(S_k:N_k),Z(S_k:N_k),'b');
hold on
plot(X_hat(S_k:N_k),Z_hat(S_k:N_k),'r');
hold on
plot(X_gps(S_k:N_k),Z_gps(S_k:N_k),'k')
legend('Viso2M','with KF','Ground truth');
%axis([-50 50 -100 500])
 xlabel('x[m]')
 ylabel('z[m]')
title('Trajectory')
saveas(gcf,[ResultDir 'DirectionmKF.jpg']);



%% error metric

for i=S_k:1:size(Tr_total_hat,2)-1
E(:,:,i)=(inv(inv(Tr_totalM{i})*Tr_totalM{i+1}))* (inv([Tr_total_GPS_P{i};0,0,0,1])*[Tr_total_GPS_P{i+1};0 0 0 1]);
E_hat(:,:,i)=(inv(inv(Tr_total_hat{i})*Tr_total_hat{i+1}))* (inv([Tr_total_GPS_P{i};0,0,0,1])*[Tr_total_GPS_P{i+1};0 0 0 1]);

end

for i=S_k:1:size(E,3)
    RMSE_Et=sqrt(sum(norm(E(1:3,4,i)))/size(E,3));
    RMSE_Er=sqrt(sum(norm(E(1:3,1:3,i)))/size(E,3));
    RMSE_Et_hat=sqrt(sum(norm(E_hat(1:3,4,i)))/size(E_hat,3));
    RMSE_Er_hat=sqrt(sum(norm(E_hat(1:3,1:3,i)))/size(E_hat,3));
end


