%Sep. 5 2017
function [ xhat ] = ExtnddKalmanFilter( N_k,S_k,H,R,z,dt,type,sigma_a,sigma_alpha )

%initalization
[x_0, P_0] = initialize_x_and_p();

xhat(:,S_k)=x_0;
P(:,:,S_k)=P_0;

for k = S_k:1:N_k    
    
    
    % EKF prediction (state and measurement prediction)
    [ xhat(:,k+1), P(:,:,k+1) ] = predict_state_and_covariance( xhat(:,k), P(:,:,k), type, sigma_a(:,k),sigma_alpha(:,k), dt );
    
    %measurment prediction
    zhat(:,k+1)=H*xhat(:,k+1);
    
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





