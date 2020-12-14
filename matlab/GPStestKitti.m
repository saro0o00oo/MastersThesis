clc;
clear all;
close all;

%need to change
sequence='04';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

% skipFrame=3;%for GPS implementation, it should be the same as demo skipFrame
% skipFrame1=1;%for data, keep it 1


load([ResultDir 'Tr']);
load([ResultDir 'Tr_total']);

% load('C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr\00\TrM');
% load('C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr\00\Tr_totalM');

%GPS
fileID = fopen(GPSdir ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
for i=1:size(Tr_total_GPS,2)
Tr_total_GPS_P{i}=reshape(Tr_total_GPS(:,i),4,3)';
end



ii=1;

for i=1:1:size(Tr_total_GPS_P,2)-1
  
    X_gps(i)=Tr_total_GPS_P{i}(1,4);
    Y_gps(i)=Tr_total_GPS_P{i}(2,4);
    Z_gps(i)=Tr_total_GPS_P{i}(3,4);
       
    X_gpsPlot(ii)=Tr_total_GPS_P{i}(1,4);
    Y_gpsPlot(ii)=Tr_total_GPS_P{i}(2,4);
    Z_gpsPlot(ii)=Tr_total_GPS_P{i}(3,4);

ii=ii+1;
end

ii=1;
for i=1:1:size(Tr_total,2)-1
  
    X(i)=Tr_total{i}(1,4);
    Y(i)=Tr_total{i}(2,4);
    Z(i)=Tr_total{i}(3,4);

    XPlot(ii)=Tr_total{i}(1,4);
    YPlot(ii)=Tr_total{i}(2,4);
    ZPlot(ii)=Tr_total{i}(3,4);

ii=ii+1;
end

ii=1;
for i=1:1:size(Tr1,3)-2

    tx(i)=Tr1(1,4,i);
    ty(i)=Tr1(2,4,i);
    tz(i)=Tr1(3,4,i);
    
    ry(i)=asin(Tr1(1,3,i));
    rz(i)=acos(Tr1(1,1,i)/cos(asin(Tr1(1,3,i))));
    rx(i)=-asin(Tr1(2,3,i)/cos(asin(Tr1(1,3,i))));
    
    txPlot(ii)=Tr1(1,4,i);
    tyPlot(ii)=Tr1(2,4,i);
    tzPlot(ii)=Tr1(3,4,i);
    
    ryPlot(ii)=asin(Tr1(1,3,i));
    rzPlot(ii)=acos(Tr1(1,1,i)/cos(asin(Tr1(1,3,i))));
    rxPlot(ii)=-asin(Tr1(2,3,i)/cos(asin(Tr1(1,3,i))));
    
    ii=ii+1;
end



ii=1;
for i=1:1:size(Tr1,3)-3

    vx(i)=tx(i+1)-tx(i);
    vy(i)=ty(i+1)-ty(i);
    vz(i)=tz(i+1)-tz(i);
    
    vxPlot(ii)=tx(i+1)-tx(i);
    vyPlot(ii)=ty(i+1)-ty(i);
    vzPlot(ii)=tz(i+1)-tz(i);
    
    ii=ii+1;
end

ii=1;
for i=1:1:size(Tr1,3)-4
    ax(i)=vx(i+1)-vx(i);
    ay(i)=vy(i+1)-vy(i);
    az(i)=vz(i+1)-vz(i);
    
    axPlot(ii)=vx(i+1)-vx(i);
    ayPlot(ii)=vy(i+1)-vy(i);
    azPlot(ii)=vz(i+1)-vz(i);
    
    ii=ii+1;
end

figure;
subplot(3,1,1);
plot(1:size(txPlot,2),txPlot);
title('tx');
subplot(3,1,2);
plot(1:size(tyPlot,2),tyPlot);
title('ty');
subplot(3,1,3);
plot(1:size(tzPlot,2),tzPlot);
title('tz');

saveas(gcf,[ResultDir 't.jpg']);

figure;
subplot(3,1,1);
plot(1:size(rxPlot,2),rxPlot);
title('rx');
subplot(3,1,2);
plot(1:size(ryPlot,2),ryPlot);
title('ry');
subplot(3,1,3);
plot(1:size(rzPlot,2),rzPlot);
title('rz');
saveas(gcf,[ResultDir 'r.jpg']);

figure;
subplot(3,1,1);
plot(1:size(vxPlot,2),vxPlot);
title('vx');
subplot(3,1,2);
plot(1:size(vyPlot,2),vyPlot);
title('vy');
subplot(3,1,3);
plot(1:size(vxPlot,2),vzPlot);
title('vz');
saveas(gcf,[ResultDir 'vt.jpg']);


figure;
subplot(3,1,1);
plot(1:size(axPlot,2),axPlot);
title('ax');
subplot(3,1,2);
plot(1:size(ayPlot,2),ayPlot);
title('ay');
subplot(3,1,3);
plot(1:size(azPlot,2),azPlot);
title('az');
saveas(gcf,[ResultDir 'at.jpg']);




figure;
subplot(3,1,1);
plot(1:size(X_gpsPlot,2),X_gpsPlot);
hold on;
plot(1:size(XPlot,2),XPlot);
title('X');
subplot(3,1,2);
plot(1:size(Y_gpsPlot,2),Y_gpsPlot);
hold on
plot(1:size(YPlot,2),YPlot);
title('Y');
subplot(3,1,3);
plot(1:size(Z_gpsPlot,2),Z_gpsPlot);
hold on
plot(1:size(ZPlot,2),ZPlot);
title('Z');
legend('GPS','ImplementS')
saveas(gcf,[ResultDir 'XYZ.jpg']);


figure;
plot(X_gpsPlot,Z_gpsPlot)
hold on
plot(XPlot,ZPlot)
legend('GPS','ImplementS')
title('Direction');
saveas(gcf,[ResultDir 'Direction.jpg']);



