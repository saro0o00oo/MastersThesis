clc;
clear all;
close all;


%need to change
sequence='12';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

skipFrame=1;%for GPS implementation, it should be the same as demo skipFrame
skipFrame1=1;%for data, keep it 1


 load([ResultDir 'TrM']);
 load([ResultDir 'Tr_totalM']);

% %GPS
% fileID = fopen(GPSdir ,'r');
% formatSpec = '%f';
% A = fscanf(fileID,formatSpec);
% Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
% for i=1:size(Tr_total_GPS,2)
% Tr_total_GPS_P{i}=reshape(Tr_total_GPS(:,i),4,3)';
% end



ii=1;

% for i=1:size(Tr_total_GPS_P,2)-1
%     %i=1:skipFrame:1000-1-skipFrame
%     X_gps(i)=Tr_total_GPS_P{i}(1,4);
%     Y_gps(i)=Tr_total_GPS_P{i}(2,4);
%     Z_gps(i)=Tr_total_GPS_P{i}(3,4);
%        
%     if i>1
%     Tr_GPS(:,:,i-1)=inv([Tr_total_GPS_P{i};0 0 0 1])*[Tr_total_GPS_P{i-1};0 0 0 1];
%     end
%     
%     X_gpsPlot(ii)=Tr_total_GPS_P{i}(1,4);
%     Y_gpsPlot(ii)=Tr_total_GPS_P{i}(2,4);
%     Z_gpsPlot(ii)=Tr_total_GPS_P{i}(3,4);
% 
% ii=ii+1;
% end

ii=1;
for i=1:skipFrame1:size(Tr_totalM,2)
  
    X(i)=Tr_totalM{i}(1,4);
    Y(i)=Tr_totalM{i}(2,4);
    Z(i)=Tr_totalM{i}(3,4);

    XPlot(ii)=Tr_totalM{i}(1,4);
    YPlot(ii)=Tr_totalM{i}(2,4);
    ZPlot(ii)=Tr_totalM{i}(3,4);

ii=ii+1;
end



ii=1;
for i=1:skipFrame1:size(TrM,3)-1

    tx(i)=TrM(1,4,i);
    ty(i)=TrM(2,4,i);
    tz(i)=TrM(3,4,i);
    
    ry(i)=asin(TrM(1,3,i));
    rz(i)=acos(TrM(1,1,i)/cos(asin(TrM(1,3,i))));
    rx(i)=-asin(TrM(2,3,i)/cos(asin(TrM(1,3,i))));
    
    txPlot(ii)=TrM(1,4,i);
    tyPlot(ii)=TrM(2,4,i);
    tzPlot(ii)=TrM(3,4,i);
    
    ryPlot(ii)=asin(TrM(1,3,i));
    rzPlot(ii)=acos(TrM(1,1,i)/cos(asin(TrM(1,3,i))));
    rxPlot(ii)=-asin(TrM(2,3,i)/cos(asin(TrM(1,3,i))));
    
    ii=ii+1;
%     
%     tx_gps(i)=Tr_GPS(1,4,i);
%     ty_gps(i)=Tr_GPS(2,4,i);
%     tz_gps(i)=Tr_GPS(3,4,i);
%     
%     ry_gps(i)=asin(Tr_GPS(1,3,i));
%     rz_gps(i)=acos(Tr_GPS(1,1,i)/cos(asin(Tr_GPS(1,3,i))));
%     rx_gps(i)=-asin(Tr_GPS(2,3,i)/cos(asin(Tr_GPS(1,3,i))));
end



% ii=1;
% for i=1:skipFrame1:size(TrM,3)-2
% 
%     vx(i)=tx(i+skipFrame1)-tx(i);
%     vy(i)=ty(i+skipFrame1)-ty(i);
%     vz(i)=tz(i+skipFrame1)-tz(i);
%     
%     vxPlot(ii)=tx(i+skipFrame1)-tx(i);
%     vyPlot(ii)=ty(i+skipFrame1)-ty(i);
%     vzPlot(ii)=tz(i+skipFrame1)-tz(i);
%     
%     ii=ii+1;
%     
%     vx_gps(i)=tx_gps(i+skipFrame1)-tx_gps(i);
%     vy_gps(i)=ty_gps(i+skipFrame1)-ty_gps(i);
%     vz_gps(i)=tz_gps(i+skipFrame1)-tz_gps(i);
% end
% 
% ii=1;
% for i=1:skipFrame1:size(TrM,3)-3
%     ax(i)=vx(i+skipFrame1)-vx(i);
%     ay(i)=vy(i+skipFrame1)-vy(i);
%     az(i)=vz(i+skipFrame1)-vz(i);
%     
%     axPlot(ii)=vx(i+skipFrame1)-vx(i);
%     ayPlot(ii)=vy(i+skipFrame1)-vy(i);
%     azPlot(ii)=vz(i+skipFrame1)-vz(i);
%     
%     ii=ii+1;
%     
%     ax_gps(i)=vx_gps(i+skipFrame1)-vx_gps(i);
%     ay_gps(i)=vy_gps(i+skipFrame1)-vy_gps(i);
%     az_gps(i)=vz_gps(i+skipFrame1)-vz_gps(i);
% end

figure;
subplot(3,1,1);
% plot(1:size(txPlot,2),txPlot);
% xlabel('Frame[k]')
% ylabel('t_x[m]')
% hold on
%plot(1:size(tx_gps,2),tx_gps);
%title('t_x');
subplot(3,1,2);
% plot(1:size(tyPlot,2),tyPlot);
% xlabel('Frame[k]')
% ylabel('t_y[m]')
% hold on
%plot(1:size(ty_gps,2),ty_gps);
%title('t_y');
subplot(3,1,3);
% plot(1:size(tzPlot,2),tzPlot);
% xlabel('Frame[k]')
% ylabel('t_z[m]')
% hold on
%plot(1:size(tz_gps,2),tz_gps);
%title('t_z');
legend('ImplementM','GPS')
saveas(gcf,[ResultDir 'tm.jpg']);

figure;
subplot(3,1,1);
% plot(1:size(rxPlot,2),rxPlot);
% xlabel('Frame[k]')
% ylabel('r_x[rad]')
% hold on;
%plot(1:size(rx_gps,2),rx_gps);
%title('rx');
subplot(3,1,2);
% plot(1:size(ryPlot,2),ryPlot);
% xlabel('Frame[k]')
% ylabel('r_y[rad]')
% hold on;
%plot(1:size(ry_gps,2),ry_gps);
%title('ry');
subplot(3,1,3);
% plot(1:size(rzPlot,2),rzPlot);
% xlabel('Frame[k]')
% ylabel('r_z[rad]')
% hold on;
%plot(1:size(rz_gps,2),rz_gps);
%title('rz');
legend('ImplementM','GPS')
saveas(gcf,[ResultDir 'rm.jpg']);

% figure;
% subplot(3,1,1);
% plot(1:size(vxPlot,2),vxPlot);
% hold on;
% plot(1:size(vx_gps,2),vx_gps);
% title('vx');
% subplot(3,1,2);
% plot(1:size(vyPlot,2),vyPlot);
% hold on;
% plot(1:size(vy_gps,2),vy_gps);
% title('vy');
% subplot(3,1,3);
% plot(1:size(vzPlot,2),vzPlot);
% hold on;
% plot(1:size(vz_gps,2),vz_gps);
% title('vz');
% legend('ImplementM','GPS')
% saveas(gcf,[ResultDir 'vtm.jpg']);


% figure;
% subplot(3,1,1);
% plot(1:size(axPlot,2),axPlot);
% hold on;
% plot(1:size(ax_gps,2),ax_gps);
% title('ax');
% subplot(3,1,2);
% plot(1:size(ayPlot,2),ayPlot);
% hold on;
% plot(1:size(ay_gps,2),ay_gps);
% title('ay');
% subplot(3,1,3);
% plot(1:size(azPlot,2),azPlot);
% hold on;
% plot(1:size(az_gps,2),az_gps);
% title('az');
% legend('ImplementM','GPS')
% saveas(gcf,[ResultDir 'atm.jpg']);




figure;
subplot(3,1,1);
plot(1:size(XPlot,2),XPlot);
% hold on;
% plot(1:size(X_gpsPlot,2),X_gpsPlot);
title('X');
subplot(3,1,2);
plot(1:size(YPlot,2),YPlot);
% hold on
% plot(1:size(Y_gpsPlot,2),Y_gpsPlot);
title('Y');
subplot(3,1,3);
plot(1:size(ZPlot,2),ZPlot);
% hold on
% plot(1:size(Z_gpsPlot,2),Z_gpsPlot);
title('Z');
legend('ImplementM','GPS')
saveas(gcf,[ResultDir 'XYZm.jpg']);

% 
figure;
plot(XPlot,ZPlot)
% hold on
% plot(X_gpsPlot,Z_gpsPlot)
% %axis([-100 100 -200 500])
% xlabel('x[m]')
% ylabel('z[m]')
% legend('ImplementM','GPS')
% title('Direction');
% saveas(gcf,[ResultDir 'Directionm.jpg']);










