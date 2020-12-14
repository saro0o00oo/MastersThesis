clc;
clear all;
close all;

load('C:\Users\alizades\Google Drive\Master\Code\libviso2\matlab\Results-Tr\2009_09_08_drive_0015\Tr');
load('C:\Users\alizades\Google Drive\Master\Code\libviso2\matlab\Results-Tr\2009_09_08_drive_0015\Tr_total');

%GPS
fileID = fopen('C:\Users\alizades\Google Drive\Master\Code\Data\httpwww.cvlibs.netdatasetskarlsruhe_sequences\2009_09_08_drive_0015\insdata.txt','r');
%2009_09_08_drive_0010
%2009_09_08_drive_0012
%2010_03_04_drive_0041
%2010_03_09_drive_0019
%2010_03_09_drive_0020
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[10,size(A,1)/10]);
%columns are: timestamp,lat,lon,alt,x,y,z,roll,pitch,yaw.
%pitch:ry   yaw:rx    roll:rz


for i=1:size(A,1)/10
    X_gps(i)=Tr_total_GPS(5,i)-Tr_total_GPS(5,1);
    Y_gps(i)=Tr_total_GPS(6,i)-Tr_total_GPS(6,1);
    Z_gps(i)=Tr_total_GPS(7,i)-Tr_total_GPS(7,1);
    rz_gps(i)=Tr_total_GPS(8,i);
    ry_gps(i)=Tr_total_GPS(9,i);
    rx_gps(i)=Tr_total_GPS(10,i);
    R_gps(:,:,i)=[cos(ry_gps(i))*cos(rz_gps(i)),-cos(ry_gps(i))*sin(rz_gps(i)),sin(ry_gps(i))...
        ;sin(rx_gps(i))*sin(ry_gps(i))*cos(rz_gps(i))+cos(rx_gps(i))*sin(rz_gps(i)),-sin(rx_gps(i))*sin(ry_gps(i))*sin(rz_gps(i))+cos(rx_gps(i))*cos(rz_gps(i)),-sin(rx_gps(i))*cos(ry_gps(i))...
        ;-cos(rx_gps(i))*sin(ry_gps(i))*cos(rz_gps(i))+sin(rx_gps(i))*sin(rz_gps(i)),cos(rx_gps(i))*sin(ry_gps(i))*sin(rz_gps(i))+sin(rx_gps(i))*cos(rz_gps(i)),cos(rx_gps(i))*cos(ry_gps(i))];
end

for i=1:size(A,1)/10-1
    tr(:,i)=[X_gps(i+1),Y_gps(i+1),Z_gps(i+1)]'-R_gps(i)*[X_gps(i),Y_gps(i),Z_gps(i)]';
end

for i=1:size(A,1)/10-1
     tx_gps(i)=tr(1,i);
     ty_gps(i)=tr(2,i);
     tz_gps(i)=tr(3,i);
%     tx_T(i)=Tr_total{i+1}(1,4)-Tr_total{i}(1,4);
%     ty_T(i)=Tr_total{i+1}(2,4)-Tr_total{i}(2,4);
%     tz_T(i)=Tr_total{i+1}(3,4)-Tr_total{i}(3,4);
    tx(i)=T_r(1,4,i);
    ty(i)=T_r(2,4,i);
    tz(i)=T_r(3,4,i);
end

for i=1:size(A,1)/10-2
    vx_gps(i)=tx_gps(i+1)-tx_gps(i);
    vy_gps(i)=ty_gps(i+1)-ty_gps(i);
    vz_gps(i)=tz_gps(i+1)-tz_gps(i);
    vx(i)=tx(i+1)-tx(i);
    vy(i)=ty(i+1)-ty(i);
    vz(i)=tz(i+1)-tz(i);
end

for i=1:size(A,1)/10-3
    ax_gps(i)=vx_gps(i+1)-vx_gps(i);
    ay_gps(i)=vy_gps(i+1)-vy_gps(i);
    az_gps(i)=vz_gps(i+1)-vz_gps(i);
    ax(i)=vx(i+1)-vx(i);
    ay(i)=vy(i+1)-vy(i);
    az(i)=vz(i+1)-vz(i);
end

figure;
subplot(3,1,1);
plot(1:size(A,1)/10,X_gps);
title('X');
subplot(3,1,2);
plot(1:size(A,1)/10,Y_gps);
title('Z');
subplot(3,1,3);
plot(1:size(A,1)/10,Z_gps);
title('Y');
figure;
subplot(3,1,1);
plot(1:size(A,1)/10,rz_gps);
title('roll');
subplot(3,1,2);
plot(1:size(A,1)/10,ry_gps);
title('pitch');
subplot(3,1,3);
plot(1:size(A,1)/10,rx_gps);
title('yaw');

figure;
subplot(3,1,1);
plot(1:size(A,1)/10-1,tx);
title('tx');
subplot(3,1,2);
plot(1:size(A,1)/10-1,ty);
title('tz');
subplot(3,1,3);
plot(1:size(A,1)/10-1,tz);
title('ty');

figure;
subplot(3,1,1);
plot(1:size(A,1)/10-2,vx);
title('vx');
subplot(3,1,2);
plot(1:size(A,1)/10-2,vy);
title('vz');
subplot(3,1,3);
plot(1:size(A,1)/10-2,vz);
title('vy');

figure;
subplot(3,1,1);
plot(1:size(A,1)/10-3,ax);
title('ax');
subplot(3,1,2);
plot(1:size(A,1)/10-3,ay);
title('az');
subplot(3,1,3);
plot(1:size(A,1)/10-3,az);
title('ay');


