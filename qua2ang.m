function angle=qua2ang(qua,roate_way)
    if nargin<2
        attitude_matrix=qua2rollpitchyaw_new(qua);
        x=asin(attitude_matrix(2,3));
        y=atan2(attitude_matrix(1,3),-attitude_matrix(3,3));
        z=atan2(attitude_matrix(2,1),-attitude_matrix(2,2));
        angle=[x y z];
    else
        attitude_matrix=qua2rollpitchyaw_new(qua);
        x=atan2(attitude_matrix(3,1),-attitude_matrix(3,2));
        y=acos(attitude_matrix(3,3));
        z=atan2(attitude_matrix(1,3), attitude_matrix(2,3));
        angle=[x y z];
    end
end