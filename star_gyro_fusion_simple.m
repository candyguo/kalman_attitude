[attitude1,attitude2,attitude3]=att_process();
load('/Users/c_c/Desktop/TLAAngVel.mat');
gyro_angvel=unique(TLAAngVel,'rows','stable');
wx=interp1(gyro_angvel(:,1),gyro_angvel(:,2),attitude2(3:end,1),'spline');
wy=interp1(gyro_angvel(:,1),gyro_angvel(:,3),attitude2(3:end,1),'spline');
wz=interp1(gyro_angvel(:,1),gyro_angvel(:,4),attitude2(3:end,1),'spline');
gyro=[attitude2(3:end,1),wx*pi/180,wy*pi/180,wz*pi/180];
anzhi_str2cam=[0.235806791780133         0.834097889533183        -0.498674215467591
     0.939162117573613        -0.327467332594395         -0.10363287026977
    -0.24973950429588        -0.443898502496165        -0.860572120477672];
gyro_xpiao=0.0000;
gyro_ypiao=-0.0000;
gyro_zpiao=-0.00000;
gyro(:,2)=gyro(:,2)-gyro_xpiao;
gyro(:,3)=gyro(:,3)-gyro_ypiao;
gyro(:,4)=gyro(:,4)-gyro_zpiao;
for i=1:length(gyro(:,1))
    qua=[attitude2(i+2,2),attitude2(i+2,3),attitude2(i+2,4),attitude2(i+2,5)];
    attitude_matrix=qua2rollpitchyaw_new(qua);
    attitude_matrix=(anzhi_str2cam'*attitude_matrix');
    q_1=rotatemat2quat(attitude_matrix,1);
    q_1=q_1/norm(q_1);
    angle_star(i,:)=[gyro(i,1) qua2ang(q_1)];
    q_mat1(i,:)=[gyro(i,1) q_1'];   
end



q_mat2(1,:)=q_mat1(1,:);
angle_gyro(1,:)=[gyro(1,1) qua2ang(q_mat2(1,2:5))];
for i=2:length(gyro(:,1))
    ou_w=[0 -gyro(i-1,2) -gyro(i-1,3) -gyro(i-1,4)
    gyro(i-1,2) 0 -gyro(i-1,4) gyro(i-1,3)
    gyro(i-1,3) gyro(i-1,4) 0 -gyro(i-1,2)
    gyro(i-1,4) -gyro(i-1,3) gyro(i-1,2) 0];
    q_dot=0.5*ou_w*[q_mat2(i-1,2),q_mat2(i-1,3),q_mat2(i-1,4),q_mat2(i-1,5)]';
    q_2=q_mat2(i-1,2:end)-q_dot'*0.5;
    q_2=q_2/norm(q_2);
    angle_gyro(i,:)=[gyro(i,1) qua2ang(q_2)];
    q_mat2(i,:)=[gyro(i,1) q_2];
end
for i=1:length(gyro(:,1))
   Q=[q_mat1(i,2) -q_mat1(i,3) -q_mat1(i,4) -q_mat1(i,5)
        q_mat1(i,3) q_mat1(i,2) -q_mat1(i,5) q_mat1(i,4)
        q_mat1(i,4) q_mat1(i,5) q_mat1(i,2) -q_mat1(i,3)
        q_mat1(i,5) -q_mat1(i,4) q_mat1(i,3) q_mat1(i,2)];
   piancha=Q*[q_mat2(i,2) -q_mat2(i,3) ...
        -q_mat2(i,4) -q_mat2(i,5)]';
    error(i,:)=piancha;
end
error_angle=piancha_angle(error)*206265;

for i=1:1420
    za=[2*(q_mat1(i,3)*q_mat1(i,5)+q_mat1(i,2)*q_mat1(i,4)) 2*(q_mat1(i,4)*q_mat1(i,5)-q_mat1(i,2)*q_mat1(i,3)) q_mat1(i,2)^2+q_mat1(i,5)^2-q_mat1(i,3)^2-q_mat1(i,4)^2];
    zb=[2*(q_mat2(i,3)*q_mat2(i,5)+q_mat2(i,2)*q_mat2(i,4)) 2*(q_mat2(i,4)*q_mat2(i,5)-q_mat2(i,2)*q_mat2(i,3)) q_mat2(i,2)^2+q_mat2(i,5)^2-q_mat2(i,3)^2-q_mat2(i,4)^2];
    star_gyro_axis_angle(i)=acos((za*zb')/(norm(za)*norm(zb)))*206265;
end
    
angle_vel_gyro(:,1)=angle_gyro(:,1);
for i=1:1420
    if(i==1420)
        angle_vel_gyro(i,2)=angle_vel_gyro(i-1,2);
        angle_vel_gyro(i,3)=angle_vel_gyro(i-1,3);
        angle_vel_gyro(i,4)=angle_vel_gyro(i-1,4);
    else
    angle_vel_gyro(i,2)=(angle_gyro(i+1,2)-angle_gyro(i,2));
    angle_vel_gyro(i,3)=(angle_gyro(i+1,3)-angle_gyro(i,3));
    angle_vel_gyro(i,4)=(angle_gyro(i+1,4)-angle_gyro(i,4));
    end
end
%gain the angle_vel and angle_star,then kalman filter,time
%intervel=0.5 second;l.l;;.llll.ll
%[x,y,z]=solve('-0.5*x*(-0.183455115781001)+(-0.5)*y*0.274040046036917+(-0.5)*(-z)*0.701972983192254=4.0859e-7*2',...
   % '-0.5*(-x)*0.701972983192254+(-0.5)*y*(-0.183455115781001)+(-0.5)*(-0.631252884769671)*z=-1.8804e-7*2',...
    %'-0.5*0.701972983192254*x+(-0.5)*(-y)*(-0.631252884769671)+(-0.5)*z*(-0.183455115781001)=6.762e-7*2')