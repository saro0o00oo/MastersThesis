function [ Err ] = calcSequenceErrors( poses_gt,poses_result,lengths,num_lengths,S_p)

step_size = 1; % every second

% pre-compute distances (from ground truth as reference)
dist = trajectoryDistances(poses_gt);

%for all start positions do
for  first_frame=S_p: step_size: size(poses_result,2)-2
    
    % for all segment lengths do
    for  i = 1:1:num_lengths
        
        % current length
        len = lengths(i);
        %
        % 			// compute last frame
        last_frame = lastFrameFromSegmentLength(dist, first_frame, len);
        %
        % 			// continue, if sequence not long enough
        if (last_frame ~= -1)
            
            
            % compute rotational and translational errors
            pose_delta_gt = inv(poses_gt{first_frame})*poses_gt{last_frame};
            pose_delta_result = inv(poses_result{first_frame})*poses_result{last_frame};
            pose_error = inv(pose_delta_result)*pose_delta_gt;
            r_err = rotationError(pose_error);
            t_err = translationError(pose_error);
         
            %compute speed
            num_frames = last_frame - first_frame + 1;
            speed = len / (0.1*num_frames);
            Err{first_frame}(1)=first_frame;
            Err{first_frame}(2) =r_err/len;
            Err{first_frame}(3) =t_err/len;
            Err{first_frame}(4)=len;
            Err{first_frame}(5)=speed;
       
        end 
    end
end

end

