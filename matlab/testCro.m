% clc
% clear all;
% close all;



for i=S_p:1:N_k-1
    
    %     ry_hat(i)=xhatEuler(2,i);
    %     rx_hat(i)=xhatEuler(1,i);
    %     rz_hat(i)=xhatEuler(3,i);
    
%     tx_hat(i)=xhat(1,i);
%     ty_hat(i)=xhat(2,i);
%     tz_hat(i)=xhat(3,i);
%     
%     ry_hat(i)=asin(xhatR_Euler(1,3,i));
     rz_hat(i)=acos(xhatR_Euler(1,1,i)/cos(asin(xhatR_Euler(1,3,i))));
%     rx_hat(i)=-asin(xhatR_Euler(2,3,i)/cos(asin(xhatR_Euler(1,3,i))));
%     
%     
%     tx_gps(i+1)=Tr_GPS(1,4,i);
%     ty_gps(i+1)=Tr_GPS(2,4,i);
%     tz_gps(i+1)=Tr_GPS(3,4,i);
%     
    ry_gps(i+1)=asin(Tr_GPS(1,3,i));
    rz_gps(i+1)=acos((Tr_GPS(1,1,i)/cos(asin(Tr_GPS(1,3,i)))));
    rx_gps(i+1)=-asin(Tr_GPS(2,3,i)/cos(asin(Tr_GPS(1,3,i))));
    
    
    sara(i)=Tr_GPS(1,1,i)/cos(asin(Tr_GPS(1,3,i)));
    sara2(i)=xhatR_Euler(1,1,i)/cos(asin(xhatR_Euler(1,3,i)));
    
    sara3(i)=acos(sara(i));
%     tx(i)=z(1,i);
%     ty(i)=z(2,i);
%     tz(i)=z(3,i);
%     
%     rx(i)=z(4,i);
%     ry(i)=z(5,i);
%     rz(i)=z(6,i);
end