function [ output ] = lastFrameFromSegmentLength(dist, first_frame, len )

	for  i = first_frame:size(dist,2)-2
		if (dist(i)<=dist(first_frame) + len)
			output= -1;
        else
            output=i;
        end
    
    end
       

end

