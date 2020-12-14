clc
clear all
close all


%need to change
sequence='04';


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
Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
for i=1:size(Tr_total_GPS,2)
    Tr_total_GPS_P{i}=reshape(Tr_total_GPS(:,i),4,3)';
    Tr_total_GPS_PP{i}=[reshape(Tr_total_GPS(:,i),4,3)';0,0,0,1];
end


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
%N_k=900;
S_k= 2; %starting frame
S_p=90;

dt=1;

Tr_total_hat{S_p}=eye(4);


%State Eq.
%X = [t_x;t_y;t_z;qw;q_x;q_t;q_z;t_x/dt;t_y/dt;t_z/dt;qw/dt;q_x/dt;q_t/dt;q_z/dt]; %state vector


%Measurment Eq.
%z=[t_x;t_y;t_z;r_x;r_y;r_z];
H=[eye(7,7) zeros(7,6)];
R           =  diag(1e-4*ones(1,7));%Measurement Noise cov. matrix


%Process noise
[ sigma_a,sigma_alpha,z ] = processNoise( sequence,z );


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
save([ResultDir 'Tr_total_hat.mat'],'Tr_total_hat')
%% Ploting

for i= S_p:1:N_k
    
    X_hat(i)=Tr_total_hat{i+1}(1,4);
    Z_hat(i)=Tr_total_hat{i+1}(3,4);
    Y_hat(i)=Tr_total_hat{i+1}(2,4);
    
    X(i)=Tr_totalM{i}(1,4)-Tr_totalM{S_p-1}(1,4);
    Z(i)=Tr_totalM{i}(3,4)-Tr_totalM{S_p-1}(3,4);
    Y(i)=Tr_totalM{i}(2,4)-Tr_totalM{S_p-1}(2,4);
    %
    %     XS(i)=Tr_total{i}(1,4)-Tr_total{S_p-1}(1,4);
    %     ZS(i)=Tr_total{i}(3,4)-Tr_total{S_p-1}(3,4);
    %     YS(i)=Tr_total{i}(2,4)-Tr_total{S_p-1}(2,4);
    
    X_gps(i)=Tr_total_GPS_P{i}(1,4)-Tr_total_GPS_P{S_p-1}(1,4);
    Y_gps(i)=Tr_total_GPS_P{i}(2,4)-Tr_total_GPS_P{S_p-1}(2,4);
    Z_gps(i)=Tr_total_GPS_P{i}(3,4)-Tr_total_GPS_P{S_p-1}(3,4);
    
    if i>1
        Tr_GPS(:,:,i-1)=inv([Tr_total_GPS_P{i};0 0 0 1])*[Tr_total_GPS_P{i-1};0 0 0 1];
    end
    
    
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
    
    tx(i)=zt(1,i);
    ty(i)=z(2,i);
    tz(i)=z(3,i);
    
    rx(i)=zr(1,i);
    ry(i)=zr(2,i);
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
legend('VISOM without KF','Our approach','GPS')
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
legend('VISOM without KF','Our approach','GPS')
saveas(gcf,[ResultDir 'rmkf.jpg']);


figure;
plot(X(timeF),Z(timeF),'b');
hold on
plot(X_hat(timeF),Z_hat(timeF),'r');
hold on
plot(X_gps(timeF),Z_gps(timeF),'k')
hold on
%plot(XS(timeF),ZS(timeF),'g')
legend('VISOM without KF','Our approach','Ground truth','Viso2S');
axis([-100 100 -50 400])
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



load([ResultDir 'Tr_total_hat']);

% for all sequences do
    
   lengths=[5,10,50,100,150,200,250,300,350,400];
   %lengths=[100,200,300,400,500,600,700,800];
   
    num_lengths = 10;
    % read ground truth and result poses
    poses_gt = Tr_total_GPS_PP(:,1:size(Tr_total_hat,2));
    poses_result = Tr_totalM(:,1:size(Tr_total_hat,2));
    
    % compute sequence errors
    seq_err = calcSequenceErrors(poses_gt, poses_result,lengths,num_lengths,S_p);
    save([ResultDir 'seq_err.mat'],'seq_err')
    for i=S_p:1:size(seq_err,2)
        r_err(i)=seq_err{i}(2);
        r_err_deg(i)=(r_err(i)*180)/pi;
        t_err(i)=seq_err{i}(3)*25;
        Len(i)=seq_err{i}(4);
    end

    
        for i=S_p:1:size(r_err_deg,2)
             a1= find (Len==400);
             a2= find (Len==350);
           a3= find (Len==300);
           a4= find (Len==250);
           a5= find (Len==200);
           a6= find (Len==150);
           a7= find (Len==100);
           a8= find (Len==50);
           a9= find (Len==10);
           a10= find (Len==5);
        end
           
         
        
%                 for i=S_p:1:size(r_err,2)
%            a1= find (Len==600);
%            a2= find (Len==500);
%            a3= find (Len==400);
%            a4= find (Len==300);
%            a5= find (Len==200);
%            a6= find (Len==100);
%                 end
%            
        
%                 
%              Len_plot=   [Len(a2(1)),Len(a3(1)),Len(a4(1)),Len(a5(1)),Len(a6(1))];
%     t_err_plot=[min(t_err(a2)),min(t_err(a3)),min(t_err(a4)),min(t_err(a5)),min(t_err(a6))];
%      r_err_plot=[min(r_err(a1)),min(r_err(a2)),min(r_err(a3)),min(r_err(a4)),min(r_err(a5)),min(r_err(a6))];
%     
    Len_plot=   [Len(a1(1)),Len(a2(1)),Len(a3(1)),Len(a4(1)),Len(a5(1)),Len(a6(1)),Len(a7(1))];
    t_err_plot=[min(t_err(a1)),min(t_err(a2)),min(t_err(a3)),min(t_err(a4)),min(t_err(a5)),min(t_err(a6)),min(t_err(a7))];
     r_err_plot=[min(r_err_deg(a1)),min(r_err_deg(a2)),min(r_err_deg(a3)),min(r_err_deg(a4)),min(r_err_deg(a5)),min(r_err_deg(a6)),min(r_err_deg(a7))];
     
     
    r_errT=sum(r_err_plot)/size(r_err_plot,2)
    t_errT=sum(t_err_plot)/size(t_err_plot,2)
    %saving the results
    
    % ground truth and result directories
    error_dir = [ResultDir  '\errors'];
    plot_path_dir = [ResultDir '\plot_path'];
    plot_error_dir = [ResultDir '\plot_error'];

    %%
    
    %% error metric



load([ResultDir 'Tr_total_hat']);

% for all sequences do
    
   lengths=[5,10,50,100,150,200,250,300,350,400];
   %lengths=[100,200,300,400,500,600,700,800];
   
    num_lengths = 10;
    % read ground truth and result poses
    poses_gt = Tr_total_GPS_PP(:,1:size(Tr_total_hat,2));
    poses_result = Tr_total_hat(:,1:size(Tr_total_hat,2));
    
    % compute sequence errors
    seq_err = calcSequenceErrors(poses_gt, poses_result,lengths,num_lengths,S_p);
    save([ResultDir 'seq_err.mat'],'seq_err')
    for i=S_p:1:size(seq_err,2)
        r_err(i)=seq_err{i}(2);
        r_err_deg(i)=(r_err(i)*180)/pi;
        t_err(i)=seq_err{i}(3)*25;
        Len(i)=seq_err{i}(4);
    end

    
        for i=S_p:1:size(r_err_deg,2)
             a1= find (Len==400);
             a2= find (Len==350);
           a3= find (Len==300);
           a4= find (Len==250);
           a5= find (Len==200);
           a6= find (Len==150);
           a7= find (Len==100);
           a8= find (Len==50);
           a9= find (Len==10);
           a10= find (Len==5);
        end
           
         
        
%                 for i=S_p:1:size(r_err,2)
%            a1= find (Len==600);
%            a2= find (Len==500);
%            a3= find (Len==400);
%            a4= find (Len==300);
%            a5= find (Len==200);
%            a6= find (Len==100);
%                 end
           
        
%                 
%              Len_plot=   [Len(a1(1)),Len(a2(1)),Len(a3(1)),Len(a4(1)),Len(a5(1)),Len(a6(1))];
%     t_errhat_plot=[min(t_err(a1)),min(t_err(a2)),min(t_err(a3)),min(t_err(a4)),min(t_err(a5)),min(t_err(a6))];
%      r_errhat_plot=[min(r_err(a1)),min(r_err(a2)),min(r_err(a3)),min(r_err(a4)),min(r_err(a5)),min(r_err(a6))];
%     
    Len_plot=   [Len(a1(1)),Len(a2(1)),Len(a3(1)),Len(a4(1)),Len(a5(1)),Len(a6(1)),Len(a7(1))];
    t_errhat_plot=[min(t_err(a1)),min(t_err(a2)),min(t_err(a3)),min(t_err(a4)),min(t_err(a5)),min(t_err(a6)),min(t_err(a7))];
     r_errhat_plot=[min(r_err_deg(a1)),min(r_err_deg(a2)),min(r_err_deg(a3)),min(r_err_deg(a4)),min(r_err_deg(a5)),min(r_err_deg(a6)),min(r_err_deg(a7))];
    
r_errhatT=sum(r_errhat_plot)/size(r_err_plot,2)
    t_errhatT=sum(t_errhat_plot)/size(t_err_plot,2)
    %saving the results
    
    % ground truth and result directories
    error_dir = [ResultDir  '\errors'];
    plot_path_dir = [ResultDir '\plot_path'];
    plot_error_dir = [ResultDir '\plot_error'];

    
    %% 
    figure
      subplot(1,2,1)
          plot(Len_plot,t_err_plot,'--bs',...
    'LineWidth',1,...
    'MarkerSize',6,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5])
 hold on
      plot(Len_plot,t_errhat_plot,'--rs',...
    'LineWidth',1,...
    'MarkerSize',6,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5])

   
  


        xlabel('Path length[m]')
    ylabel('Translation error[%]')
    legend('VISOM without KF','our approach')

    
    subplot(1,2,2)    
    plot(Len_plot,r_err_plot,'--bs',...
    'LineWidth',1,...
    'MarkerSize',6,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5])


hold on
    
        plot(Len_plot,r_errhat_plot,'--rs',...
    'LineWidth',1,...
    'MarkerSize',6,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5])
   

    xlabel('Path length[m]')
    ylabel('Rotation error[deg/m]')
    legend('VISOM without KF','our approach')
    

    