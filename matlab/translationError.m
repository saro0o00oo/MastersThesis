function [ transErr ] = translationError(pose_error )

     dx = pose_error(1,4);
	 dy = pose_error(2,4);
	 dz = pose_error(3,4);
	transErr= sqrt(dx*dx + dy*dy + dz*dz);


end

