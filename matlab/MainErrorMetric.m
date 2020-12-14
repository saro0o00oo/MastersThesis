clc;
clear all;
close all;

sequence='06';


ResultDir=['C:\Users\alizades\Google Drive\Master\Code\libviso2 - Copy\matlab\Results-Tr-fig\' sequence '\'];
DataDir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\dataset\sequences\' sequence '\'];
GPSdir=['C:\Users\alizades\Google Drive\Master\Code\Data\data_odometry_gray-httpwww.cvlibs.netdatasetskittieval_odometry.php\data_odometry_poses\dataset\poses\' sequence '.txt'];

%GPS
fileID = fopen(GPSdir ,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
Tr_total_GPS=reshape(A,[12,size(A,1)/12]);
for i=1:size(Tr_total_GPS,2)
    Tr_total_GPS_P{i}=reshape(Tr_total_GPS(:,i),4,3)';
end

load([ResultDir 'TrM']);
load([ResultDir 'Tr_totalM']);

saveas(gcf,[ResultDir 'XYZm.jpg']);
	% ground truth and result directories
	error_dir = [ResultDir  '\errors'];
	plot_path_dir = [ResultDir '\plot_path'];
	plot_error_dir = [ResultDir '\plot_error'];
    

	% for all sequences do
	for i = 11:1:22


		% read ground truth and result poses
		 poses_gt = Tr_total_GPS_P;
		 poses_result = load([ResultDir 'TrM']);

		% compute sequence errors    
		 seq_err = calcSequenceErrors(poses_gt, poses_result);
        save([ ResultDir 'seq_err.mat'],'seq_err')

		% add to total errors
		%total_err.insert(total_err.end(), seq_err.begin(), seq_err.end());
    end
			