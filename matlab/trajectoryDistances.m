function [ dist ] = trajectoryDistances( poses )

dist(1)=0;
	for  i = 1:1:size(poses,2)-1
        
		P1 = poses{i};
		 P2 = poses{i+1};
		 dx = P1(1,4) - P2(1,4);
		 dy = P1(2,4) - P2(2,4);
		 dz = P1(3,4) - P2(3,4);
		dist(i+1)=dist(i) + sqrt(dx*dx + dy*dy + dz*dz);
    end
end

