clc
clear all
%% Parameters

MC          =50; %num of Monto Carlo runs
N_k         = 200; %num of frames
%Sensor
Ts          = 1; %Sampling Time
Pd          = 0.9; %probability of detection
x_s         = [0 0 0 0]; %[x_s xd_s y_s yd_s] Sensor Position & velocity
sigma_rng   = 10; %Error Standard Deviation range
sigma_azi   = 0.01; %Error Standard Deviation azimuth
Lambda      = 1e-4; %False Alarm Density
%Coverage.range=0 to 100000;
%Coverage.Azimuth=-pi to pi;
Vv          = 2*pi*10000; %volume of validation region

x_0 = [10000;-30;100;-1];%initial true value
%State Eq.
%Single Target with Constant Velocity
%X = [x;xd;y;yd]; %state vector
ns          = 4;%num of states
%x=ones(4,1);
F           = [1 Ts 0 0;0 1 0 0;0 0 1 Ts;0 0 0 1]; %system matrix
G           = [(Ts^2)/2 0;Ts 0;0 (Ts^2)/2;0 Ts]; %noise gain
q           = [0.0001 0; 0 0.0001]'; %Process Noise var.
Q           = (G*q*G'); %Process Noise cov. matrix

%Measurment Eq.
%Range & Azimuth
%h(1,k)      = [sqrt((x(1,k)-x_s(1)).^2 + (x(3,k)-x_s(2)).^2)];
%h(2,k)      = [atan2((x(3,k)-x_s(2)),(x(1,k)-x_s(1)))];
%z=[rng;azi];
nm          = 2;%num of measurments
%Jaccobian:
%H(k)=[2*(x(1,k)-x_s)*((x(1,k)-x_s)^2+(x(3,k)-y_s)^2)^-.5  0  2*(x(3,k)-y_s)*((x(1,k)-x_s)^2+(x(3,k)-y_s)^2)^-.5  0;...
%    -((((x(1,k)-x_s)^2)/(x(3,k)-y_s))+(x(3,k)-y_s))^-1  0  ((x(1,k)-x_s)*(1+((x(3,k)-y_s)/(x(1,k)-x_s))^2))^-1  0];
r           = [sigma_rng^2;sigma_azi^2]; %Measurement Noise var.
R           = diag(r); %Measurement Noise cov. matrix

% Tracker
%Nearest Neighbor data association

Gate        = chi2inv(.95,2); %validation gate

%Initialization
%Track is already initialized
P(:,:,1)    = diag([1000,10,1000,10]);

%Performance evaluation
%RMSE


%% Extended Kalman Filter
%Feb. 20, 2016


%evolution of the system
for m=1:MC
    %initalization
    x(:,1)=x_0;
    xhat(:,1)=mvnrnd(x(:,1), P(:,:,1));%sqrt(diag(P(:,:,1))).*randn + x(:,1);%xhat(0|0)=normal distribution with mean=x(0)&var.=P(0|0)
    
    for k = 1:1:N_k
        l=0;%tmp param(num of misdetection)
        
        %state eq.
        x(:,k+1) = F*x(:,k) + G*mvnrnd(zeros(2,1),q)';
        %measurment noise
        w = mvnrnd(zeros(2,1), R)';%r*randn;
        %measurment eq.
        %z(:,k+1) = h(k+1)@x(k+1) + w;
        z(:,k+1)=[sqrt((x(1,k+1)-x_s(1)).^2 + (x(3,k+1)-x_s(2)).^2);...
            atan2((x(3,k+1)-x_s(2)),(x(1,k+1)-x_s(1)))] + w;
        
        
        %false alarm
        N_FA = poissrnd(Lambda*Vv);
        if N_FA==0
            FA = [];
        else
            FA = zeros(2,N_FA);
            FA(1,:)=unifrnd(0,10000,1,N_FA);
            FA(2,:)=unifrnd(-pi,pi,1,N_FA);
            %10000*rand(1,8);
            %2*pi*rand(1,8)-pi;
        end
        
        meas = zeros(2,0);
        meas = [meas FA];
        D_flag=rand<.9;%1:detection,0:misdetection
        if (D_flag == 1)
            meas(:, end+1) = z(:,k+1);
        end
        
        %estimation of the state
        
        %state prediction
        xhat(:,k+1) = F*xhat(:,k); %xhat(k+1|k)
        %measurment prediction
        %zhat(:,k+1)=h@xhat(k+1); %zhat(k+1|k)
        zhat(:,k+1)=[sqrt((xhat(1,k+1)-x_s(1)).^2 + (xhat(3,k+1)-x_s(2)).^2);...
            atan2((xhat(3,k+1)-x_s(2)),(xhat(1,k+1)-x_s(1)))];
        
        %state covariance computation(offline)
        %state prediction cov.
        %Jaccobian evaluation, H(k+1)@xhat(k+1)
        H=[(xhat(1,k+1)-x_s(1))*((xhat(1,k+1)-x_s(1))^2+(xhat(3,k+1)-x_s(2))^2)^-.5  0  (xhat(3,k+1)-x_s(2))*((xhat(1,k+1)-x_s(1))^2+(xhat(3,k+1)-x_s(2))^2)^-.5  0;-((((xhat(1,k+1)-x_s(1))^2)/(xhat(3,k+1)-x_s(2)))+(xhat(3,k+1)-x_s(2)))^-1  0  ((xhat(1,k+1)-x_s(1))*(1+((xhat(3,k+1)-x_s(2))/(xhat(1,k+1)-x_s(1)))^2))^-1  0];
        %    H=zeros(2,4);
        %u can use H instead of H(:,:,k+1)
        P(:,:,k+1) = F*P(:,:,k)*F' + Q;%P(k+1|k)
        %innovation covariance
        S(:,:,k+1)=H*P(:,:,k+1)*H' + R;
        %filter gain
        W(:,:,k+1)=P(:,:,k+1)*H'*inv(S(:,:,k+1));
        
        j = 1;%tmp parameter
        gated_meas=zeros(2,0);
        [m1,m2]=size(meas);
        for i=1:m2
            if (zhat(:,k+1)-meas(:,i))'*inv(S(:,:,k+1))'*(zhat(:,k+1)-meas(:,i))<Gate
                % && (zhat(2,k+1)-FA(2,i))'*inv(S(k+1))'*(zhat(2,k+1)-FA(2,i))<Gate
                gated_meas(:,j)=meas(:,i);
                j=j+1;
            end
        end
        
        %Nearest Neighbor
        z_asso=NN(gated_meas,zhat(:,k+1),S(:,:,k+1),D_flag);
        
        if isempty(z_asso)%no measurment(misdetection)
            
            %updated state cov.
            P(:,k+1)=P(:,k+1);%P(k+1|k+1)
            
            %updated state estimate
            xhat(:,k+1)=xhat(:,k+1);%xhat(k+1|k+1)
            l=l+1;%num of misdetection
            
        else            
            %measurment residual(innovation)
            V(:,k+1)=z_asso-zhat(:,k+1);
            
            %updated state cov.
            P(:,:,k+1)=P(:,:,k+1) - W(:,:,k+1)*S(:,:,k+1)*W(:,:,k+1)';%P(k+1|k+1)
                        
            %updated state estimate
            xhat(:,k+1)=xhat(:,k+1) + W(:,:,k+1)*V(:,k+1);%xhat(k+1|k+1)           
        end    
       %error in each M.C. runs and each frames
       x_err(k,m)=(x(1,k+1)-xhat(1,k+1))^2+(x(3,k+1)-xhat(3,k+1))^2;        
    end
       
end

%RMSE    (state prediction error)
 x_rmse = sqrt(sum(x_err, 2)/MC);

%% Ploting

figure;
plot(x(1,:),x(3,:),'r+');
hold all;
plot(xhat(1,:),xhat(3,:),'g--');
grid on;
legend('true state','estimated state')
title('true and estimated state values');
xlabel('x');
ylabel('y');

%RMSE ploting
figure
plot(x_rmse);
xlabel('k');
ylabel('RMSE');





