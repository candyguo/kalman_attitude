function qua=ang2qua(ang)
    x=ang(1);
    y=ang(2);
    z=ang(3);
    attitude_mat(1,1)=cos(y)*cos(z)-sin(x)*sin(y)*sin(z);
    attitude_mat(1,2)=cos(y)*sin(z)+sin(x)*sin(y)*cos(z);
    attitude_mat(1,3)=-cos(x)*sin(y);
    attitude_mat(2,1)=-cos(x)*sin(z);
    attitude_mat(2,2)=cos(x)*cos(z);
    attitude_mat(2,3)=sin(x);
    attitude_mat(3,1)=sin(y)*cos(z)+sin(x)*cos(y)*sin(z);
    attitude_mat(3,2)=sin(y)*sin(z)-sin(x)*cos(y)*cos(z);
    attitude_mat(3,3)=cos(x)*cos(y);
    qua=rotatemat2quat(attitude_mat,1);
end