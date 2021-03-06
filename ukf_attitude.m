Nsamples=180;
dt=1;
n=6;
m=3;
b1=1/206265;b2=b1;b3=b2;
P = 1e-6*eye(6);
w_start=[1*pi/180 0 0];%rad/s
MeasNoise = 0.00002;
gyro_noise=1/206265;
xhat=[1 0 0 0 0 0 0]';
xtrue=[1 0 0 0]';
xguzhi=[1 0 0 0]';
xmeasure=[0 0 0]';
xdotNoise = [1/206265 0.1/206265];
Q = [xdotNoise(1)^2 0 0 0 0 0
     0 xdotNoise(1)^2 0 0 0 0
     0 0 xdotNoise(1)^2 0 0 0
     0 0 0 xdotNoise(2)^2 0 0
     0 0 0 0 xdotNoise(2)^2 0
     0 0 0 0 0 xdotNoise(2)^2];
R = [4e-10 0 0; 0 4e-10 0;0 0 4e-10];
xhatArray = [];
xguzhiArray=[];
xmeasureArray=[];
xtrueArray=[];
tArray = [];
for t=1:dt:Nsamples
    xhatArray = [xhatArray xhat];
    xguzhiArray=[xguzhiArray xguzhi];
    xtrueArray=[xtrueArray xtrue];
    xmeasureArray=[xmeasureArray xmeasure];
    tArray = [tArray t];
    w_gyro=[w_start(1)+gyro_noise*randn+b1 gyro_noise*randn+b2 gyro_noise*randn+b3];%rad/s
    ou_wtrue=[0 -w_start(1) -w_start(2) -w_start(3)
    w_start(1) 0 -w_start(3) w_start(2)
    w_start(2) w_start(3) 0 -w_start(1)
    w_start(3) -w_start(2) w_start(1) 0];

    ou_w=[0 -w_gyro(1) -w_gyro(2) -w_gyro(3)
    w_gyro(1) 0 -w_gyro(3) w_gyro(2)
    w_gyro(2) w_gyro(3) 0 -w_gyro(1)
    w_gyro(3) -w_gyro(2) w_gyro(1) 0];

    ou_what=[0 -(w_start(1)+xdotNoise(1)*randn+b1-xhat(5)) -(w_start(2)+xdotNoise(1)*randn+b2-xhat(6)) -(w_start(3)+xdotNoise(1)*randn+b3-xhat(7))
    w_start(1)+xdotNoise(1)*randn+b1-xhat(5) 0 -(w_start(3)+xdotNoise(1)*randn+b3-xhat(7)) (w_start(2)+xdotNoise(1)*randn+b2-xhat(6))
    w_start(2)+xdotNoise(1)*randn+b2-xhat(6) w_start(3)+xdotNoise(1)*randn+b3-xhat(7) 0 -(w_start(1)+xdotNoise(1)*randn+b1-xhat(5))
    w_start(3)+xdotNoise(1)*randn+b3-xhat(7) -(w_start(2)+xdotNoise(1)*randn+b2-xhat(6)) w_start(1)+xdotNoise(1)*randn+b1-xhat(5) 0];

    xtrue=xtrue+dt*0.5*ou_wtrue*[xtrue(1),xtrue(2),xtrue(3),xtrue(4)]';
    xtrue=xtrue/norm(xtrue);
    xguzhi=xguzhi+0.5*dt*ou_w*[xguzhi(1) xguzhi(2) xguzhi(3) xguzhi(4)]';
    xguzhi=xguzhi/norm(xguzhi);
    xhat(1:4,1)=xhat(1:4,1)+0.5*ou_what*[xhat(1),xhat(2),xhat(3),xhat(4)]'*dt;
    xhat(1:4)=xhat(1:4)/norm(xhat(1:4));
    xmeasure=[xtrue(2)+MeasNoise*randn xtrue(3)+MeasNoise*randn xtrue(4)+MeasNoise*randn]';
    fxi=zeros(n,2*n+1);
    x=[0 0 0 0 0 0]';
    [Xi,W]=SigmaPoints(x,P,0);
    F = [0 w_start(3)+xdotNoise(1)*randn+b3-xhat(7) -(w_start(2)+xdotNoise(1)*randn+b2-xhat(6)) -0.5 0 0
        -(w_start(3)+xdotNoise(1)*randn+b3-xhat(7)) 0 w_start(1)+xdotNoise(1)*randn+b1-xhat(5) 0 -0.5 0
        w_start(2)+xdotNoise(1)*randn+b2-xhat(6) -(w_start(1)+xdotNoise(1)*randn+b1-xhat(5)) 0 0 0 -0.5
        0 0 0 0 0 0
        0 0 0 0 0 0
        0 0 0 0 0 0];
    fai_transmat=eye(6)+F*dt+(dt^2/2)*F*F;
    for i=1:2*n+1
        fxi(:,i)=fai_transmat*Xi(:,i);
    end
    [xp,Pp]=UT(fxi,W,Q);
    hxi=zeros(m,2*n+1);
    for j=1:2*n+1
        hxi(:,j)=[fxi(1,j),fxi(2,j),fxi(3,j)];
    end
    [zp,Pz]=UT(hxi,W,R);
    Pxz=zeros(n,m);
    for l=1:2*n+1
        Pxz=Pxz+W(l)*(fxi(:,l)-xp)*(hxi(:,l)-zp)';
    end
    K=Pxz*inv(Pz);
    z=[xmeasure(1)-xhat(2);xmeasure(2)-xhat(3);xmeasure(3)-xhat(4)];
    x=xp+K*(z-zp);
    P=Pp-K*Pz*K';
    Q_mul=[xhat(1) -xhat(2) -xhat(3) -xhat(4)
        xhat(2) xhat(1) -xhat(4) xhat(3)
        xhat(3) xhat(4) xhat(1) -xhat(2)
        xhat(4) -xhat(3) xhat(2) xhat(1)];
    xhat(1:4,1)=Q_mul*[sqrt(1-x(1)^2-x(2)^2-x(3)^2) x(1) x(2) x(3)]';
    xhat(1:4)=xhat(1:4)/norm(xhat(1:4));
    xhat(5:7,1)=xhat(5:7,1)+[x(4) x(5) x(6)]';
end
figure;
plot(tArray,xhatArray(1,:),'r:')
grid on;
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('q0');
figure;
plot(tArray,xhatArray(2,:),'r:')
grid on;
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('q1');
figure;
plot(tArray,xhatArray(3,:),'r:')
grid on;
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('q2');

figure;
plot(tArray,xhatArray(4,:),'r:')
grid on;
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('q3');

figure;
plot(tArray,xhatArray(5,:),'r:')
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('b1');

figure;
plot(tArray,xhatArray(6,:),'r:')
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('b2');

figure;
plot(tArray,xhatArray(7,:),'r:')
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('b3');

figure
plot(xguzhiArray(end,:))
hold on;
plot(xhatArray(4,:))
hold on;
plot(xmeasureArray(end,:))
hold on;
plot(xtrueArray(end,:))
