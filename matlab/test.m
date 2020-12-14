

close all;

for aa=1:size(Im,3)
    imshow(Im(:,:,aa))
end


 tt=1:size(Pose1,3);
 
 for ss=1:size(Pose1,3)
     X1(ss)=Pose1(1,4,ss);
     Y1(ss)=Pose1(2,4,ss);
     Z1(ss)=Pose1(3,4,ss);
     
          X2(ss)=Pose2(1,4,ss);
     Y2(ss)=Pose2(2,4,ss);
     Z2(ss)=Pose2(3,4,ss);
 end
 
figure;
plot(tt,X1,'r');
hold on;
plot(tt,X2,'b');
title('x');

figure;
plot(tt,Y1,'r');
hold on;
plot(tt,Y2,'b');
title('y');

figure;
plot(tt,Z1,'r');
hold on;
plot(tt,Z2,'b');
title('z');

figure;
plot(X1,Y1);
title('direction');

% figure;
% plot(X1,Z1);
% title('direction2');
%a=0;

% figure;
% plot(tt,T1(1,:),'r');
% hold on;
% plot(tt,T2(1,:),'b');
% title('x');
% 
% figure;
% plot(tt,T1(2,:),'r');
% hold on;
% plot(tt,T2(2,:),'b');
% title('y');
% 
% figure;
% plot(tt,T1(3,:),'r');
% hold on;
% plot(tt,T2(3,:),'b');
% title('z');