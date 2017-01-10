function m = qua2rollpitchyaw_new( q )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% clc
% q = [q0, q1, q2, q3];

m = zeros(3,3);
m(1,1) = q(1)^2+q(2)^2+q(3)^2+q(4)^2 - 2*(q(3)^2 + q(4)^2);
m(1,2) = 2*(q(2)*q(3) - q(1)*q(4));
m(1,3) = 2*(q(1)*q(3) + q(2)*q(4));

m(2,1) = 2*(q(2)*q(3) + q(1)*q(4));
m(2,2) = q(1)^2+q(2)^2+q(3)^2+q(4)^2 - 2*(q(2)^2 + q(4)^2);
m(2,3) = 2*(q(3)*q(4) - q(1)*q(2));

m(3,1) = 2*(q(2)*q(4) - q(1)*q(3));
m(3,2) = 2*(q(1)*q(2) + q(3)*q(4));
m(3,3) = q(1)^2+q(2)^2+q(3)^2+q(4)^2 - 2*(q(2)^2 + q(3)^2);

end

