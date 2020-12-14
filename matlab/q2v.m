function [v] = q2v( q )

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

rx=atan2((q3*q4+q1*q2),.5-(q2^2+q3^2));
ry=asin(-2*(q2*q4-q1*q3));
rz=atan2((q2*q3+q1*q4),.5-(q3^2+q4^2));

v=[rx;ry;rz];
end