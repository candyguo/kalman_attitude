function q= rotatemat2quat(C,way)
%C=WANG'S quan rotatemat
    if way==1
    K=1/3*[C(1,1)-C(2,2)-C(3,3) C(2,1)+C(1,2) C(3,1)+C(1,3) C(2,3)-C(3,2)
    C(2,1)+C(1,2) C(2,2)-C(1,1)-C(3,3) C(3,2)+C(2,3) C(3,1)-C(1,3)
    C(3,1)+C(1,3) C(3,2)+C(2,3) C(3,3)-C(1,1)-C(2,2) C(1,2)-C(2,1)
    C(2,3)-C(3,2) C(3,1)-C(1,3) C(1,2)-C(2,1) C(1,1)+C(2,2)+C(3,3)];
    [vec,value]=eig(K);
    temp=diag(value);
    ind=find(temp==max(temp(:)));
    max_vec=vec(:,ind);
    q=[max_vec(4) -max_vec(1) -max_vec(2) -max_vec(3)]';
    else
        q0=0.5*sqrt(trace(C)+1);
        q1=(C(2,3)-C(3,2))/(4*q0);
        q2=(C(3,1)-C(1,3))/(4*q0);
        q3=(C(1,2)-C(2,1))/(4*q0);
        q=[q0 q1 q2 q3];
    end
end

