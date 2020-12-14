
clc
clear all
close all


%vidObj = VideoReader('C:\Users\alizades\Google Drive\Master\IR\20170327-160818-02-20170327-162318-N (online-video-cutter.com).mp4');


% nFrames=vidObj.NumberOfFrames;
% frameRate=vidObj.FrameRate;
% for i=1:nFrames %first 10 seconds=240 frames
%     Frame(:,:,:,i)=read(vidObj,i);
%     Frameg(:,:,i)=rgb2gray(Frame(:,:,:,i));
%     %imshow(Frame(:,:,i));
% end

%load('Frameg1.mat')

for i=1:10:size(Frameg,3)
tic
F_back  =Frameg(:,:,i);

F_for = Frameg(:,:,i+5);

ptsOriginal  = detectSURFFeatures(F_back,'MetricThreshold',6000);
ptsDistorted = detectSURFFeatures(F_for,'MetricThreshold',6000);
[featuresOriginal,validPtsOriginal] = extractFeatures(F_back,ptsOriginal);
[featuresDistorted,validPtsDistorted] = extractFeatures(F_for,ptsDistorted);

index_pairs = matchFeatures(featuresOriginal,featuresDistorted);
matchedPtsOriginal  = validPtsOriginal(index_pairs(:,1));
matchedPtsDistorted = validPtsDistorted(index_pairs(:,2));

[tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(matchedPtsDistorted,matchedPtsOriginal,'affine');

outputView = imref2d(size(F_back));
Ir = imwarp(F_for,tform,'outputView',outputView);
%Ir2 = imwarp(F_for,tform);
%Ir3 = imwarp(F_back,tform);

% figure(1);
% imshow(Frameg(:,:,50));
% figure(2);
% imshow(Frameg(:,:,100));
% 
% for iii=1:size(Frameg(:,:,50),1)
%     for jjj=1:size(Frameg(:,:,50),2)
%         Frameg(iii+205,jjj,100)=Frameg(iii,jjj,50);
%     end
% end
% 
% figure(3);
% imshow(Frameg(:,:,50));
% figure(4);
% imshow(Frameg(:,:,100));

sub=F_for-F_back;
sub_predict=F_back-Ir;

%sub=F_for-F_back;

thres = 30;
for i=1:size(sub,1)
    for j=1:size(sub,2)
       if (sub (i,j) > thres)
           sub(i,j) = 255;
       elseif (int16(F_for (i,j)) - int16(F_back (i,j))  < -thres)
           sub(i,j) = 255;
       else
           sub(i,j) = 0;
       end
    end
end
for i=1:size(sub_predict,1)
    for j=1:size(sub_predict,2)
       if (sub_predict (i,j) > thres)
           sub_predict(i,j) = 255;
       elseif (int16(F_back (i,j)) - int16(Ir (i,j))  < -thres)
           sub_predict(i,j) = 255;
       else
           sub_predict(i,j) = 0;
       end
    end
end
figure(1);
%subplot(1,2,1)
imshow(sub)
figure(2);
%subplot(1,2,2)
imshow(sub_predict)
 %figure(3);
%  subplot(1,3,3)
%  imshow(Frameg(:,:,i))
toc



waitforbuttonpress;
end

tic

F_back  =Frameg(:,:,190);
imshow(F_back);
title('Base image');
%distorted = imresize(original,0.7);
F_for = Frameg(:,:,200);
figure; imshow(F_for);
title('Transformed image');

ptsOriginal  = detectSURFFeatures(F_back,'MetricThreshold',10000);
ptsDistorted = detectSURFFeatures(F_for,'MetricThreshold',10000);
[featuresOriginal,validPtsOriginal] = extractFeatures(F_back,ptsOriginal);
[featuresDistorted,validPtsDistorted] = extractFeatures(F_for,ptsDistorted);

index_pairs = matchFeatures(featuresOriginal,featuresDistorted);
matchedPtsOriginal  = validPtsOriginal(index_pairs(:,1));
matchedPtsDistorted = validPtsDistorted(index_pairs(:,2));
figure;
showMatchedFeatures(F_back,F_for,matchedPtsOriginal,matchedPtsDistorted);
title('Matched SURF points,including outliers');

[tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(matchedPtsDistorted,matchedPtsOriginal,'affine');
figure;

% 
%  [s*cos(ang)  s*-sin(ang)  0;
%   s*sin(ang)   s*cos(ang)  0;
%          t_x         t_y   1]
% 
% 
H(:,:) = tform.T;
R = H(1:2,1:2);
% Compute theta from mean of two possible arctangents
theta = mean([atan2(R(2),R(1)) atan2(-R(3),R(4))]);
% Compute scale from mean of two stable mean calculations
scale = mean(R([1 4])/cos(theta));
%scale=1;
% Translation remains the same:
translation = H(3, 1:2);
t_x=translation(1);
t_y=translation(2);
% Reconstitute new s-R-t transform:
HsRt(:,:) = [[scale*[cos(theta) -sin(theta); sin(theta) cos(theta)];translation], [0 0 1]'];

showMatchedFeatures(F_back,F_for,inlierPtsOriginal,inlierPtsDistorted);
title('Matched inlier points');

T2=  [scale*cos(theta)   scale*-sin(theta)  0;
          scale*sin(theta)   scale*cos(theta)   0;
          t_x              t_y             1];
      
tform.T=T2;
outputView = imref2d(size(F_back));
Ir = imwarp(F_for,tform,'outputView',outputView);
figure; imshow(Ir);
title('Recovered image');

sub_predict=F_back-Ir;
figure;
imshow(sub_predict)

sub=F_for-F_back;
figure;
imshow(sub)
toc