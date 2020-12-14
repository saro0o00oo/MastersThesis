%Aug. 18 2017
function [ xhat ] = KalmanFilter( N_k,S_k,x_0,P_0,F,H,Q,R,z )
%A simple Kalman filter

%evolution of the system

    %initalization
    x(:,S_k)=x_0;
    xhat(:,S_k)=x(:,1);%mvnrnd(x(:,1), P_0);%sqrt(diag(P(:,:,1))).*randn + x(:,1);%xhat(0|0)=normal distribution with mean=x(0)&var.=P(0|0)
    P(:,:,S_k)=P_0;
    for k = S_k:1:N_k
        
        %estimation of the state
        
        %state prediction
        xhat(:,k+1) = F*xhat(:,k); %xhat(k+1|k)
        
        %measurment prediction
        zhat(:,k+1)=H*xhat(:,k+1);
        
        
        P(:,:,k+1) = F*P(:,:,k)*F' + Q;%P(k+1|k)
        %innovation covariance
        S(:,:,k+1)=H*P(:,:,k+1)*H' + R;
        %filter gain
        W(:,:,k+1)=P(:,:,k+1)*H'*inv(S(:,:,k+1));
        
        
        %measurment residual(innovation)
        V(:,k+1)=z(:,k+1)-zhat(:,k+1);
        
        %updated state cov.
        P(:,:,k+1)=P(:,:,k+1) - W(:,:,k+1)*S(:,:,k+1)*W(:,:,k+1)';%P(k+1|k+1)
        
        %updated state estimate
        xhat(:,k+1)=xhat(:,k+1) + W(:,:,k+1)*V(:,k+1);%xhat(k+1|k+1)
        
    end
    




