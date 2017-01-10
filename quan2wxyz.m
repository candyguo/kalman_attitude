function [wx wy wz]=quan2wxyz()
clc
[attitude1,attitude2,attitude3]=att_process();
anzhi_str2cam=[0.235806791780133         0.834097889533183        -0.498674215467591
     0.939162117573613        -0.327467332594395         -0.10363287026977
    -0.24973950429588        -0.443898502496165        -0.860572120477672];
for i=1:length(attitude2(:,1))
    qua=[attitude2(i,2),attitude2(i,3),attitude2(i,4),attitude2(i,5)];
    attitude_matrix=qua2rollpitchyaw_new(qua);
    attitude_matrix=(anzhi_str2cam'*attitude_matrix');
    q=rotatemat2quat(attitude_matrix,1);
    attitude_body(i,:)=q;
end
attitude_vel=-diff(attitude_body);
attitude_vel(end+1,:)=attitude_vel(end,:);
wx=4*(attitude_body(:,4).*attitude_vel(:,1)+attitude_body(:,3).*attitude_vel(:,2)-...
    attitude_body(:,2).*attitude_vel(:,3)-attitude_body(:,1).*attitude_vel(:,4))*180/pi;
wy=4*(-attitude_body(:,3).*attitude_vel(:,1)+attitude_body(:,4).*attitude_vel(:,2)+...
    attitude_body(:,1).*attitude_vel(:,3)-attitude_body(:,2).*attitude_vel(:,4))*180/pi;
wz=4*(attitude_body(:,2).*attitude_vel(:,1)-attitude_body(:,1).*attitude_vel(:,2)+...
    attitude_body(:,4).*attitude_vel(:,3)-attitude_body(:,3).*attitude_vel(:,4))*180/pi;
end