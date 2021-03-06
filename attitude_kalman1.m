clear
clc
w_start=[1*pi/180 0 0];%rad/s
MeasNoise = 0.0001; % standard deviation of measurement noise
R = [1e-8 0 0; 0 1e-8 0;0 0 1e-8]; % Measurement noise covariance
xdotNoise = [1/206265 0.0/206265];
Q = [xdotNoise(1)^2 0 0 0 0 0
    0 xdotNoise(1)^2 0 0 0 0
    0 0 xdotNoise(1)^2 0 0 0
    0 0 0 xdotNoise(2)^2 0 0
    0 0 0 0 xdotNoise(2)^2 0
    0 0 0 0 0 xdotNoise(2)^2]; % Process noise covariance
P = 1e-6*eye(6); % Initial state estimation covariance
b3 = 0.00/206265; b2 = b3; b1 = b2;
dt = 1; % Integration step size
tf = 360; % Simulation length
x = [0 0 0 0 0 0]'; % Initial state
xhat = [1 0 0 0 0 0 0]'; % State estimate
xtrue=[1 0 0 0]';
xguzhi=[1 0 0 0]';
xmeasure=[0 0 0]';
dtPlot = 1; % How often to plot results
% Initialize arrays for plotting at the end of the program
xArray = [];
xhatArray = [];
trPArray = [];
xguzhiArray=[];
xmeasureArray=[];
xtrueArray=[];
tArray = [];
% Begin simulation loop
for t = 0 : dt : tf
    xArray = [xArray x];
    xhatArray = [xhatArray xhat];
    xguzhiArray=[xguzhiArray xguzhi];
    xmeasureArray=[xmeasureArray xmeasure];
    xtrueArray=[xtrueArray xtrue];
    trPArray = [trPArray trace(P)];
    tArray = [tArray t];
    ou_w=[0 -(w_start(1)+xdotNoise(1)*randn+b1) -(w_start(2)+xdotNoise(1)*randn+b2) -(w_start(3)+xdotNoise(1)*randn+b3)
    w_start(1)+xdotNoise(1)*randn+b1 0 -(w_start(3)+xdotNoise(1)*randn+b3) (w_start(2)+xdotNoise(1)*randn+b2)
    w_start(2)+xdotNoise(1)*randn+b2 w_start(3)+xdotNoise(1)*randn+b3 0 -(w_start(1)+xdotNoise(1)*randn+b1)
    w_start(3)+xdotNoise(1)*randn+b3 -(w_start(2)+xdotNoise(1)*randn+b2) w_start(1)+xdotNoise(1)*randn+b1 0];
    
    ou_wtrue=[0 -w_start(1) -w_start(2) -w_start(3)
    w_start(1) 0 -w_start(3) w_start(2)
    w_start(2) w_start(3) 0 -w_start(1)
    w_start(3) -w_start(2) w_start(1) 0];
    xtrue=xtrue+dt*0.5*ou_wtrue*[xtrue(1),xtrue(2),xtrue(3),xtrue(4)]';
    xtrue=xtrue/norm(xtrue);
    xhat(1:4,1)=xhat(1:4,1)+0.5*ou_w*[xhat(1),xhat(2),xhat(3),xhat(4)]'*dt;
    xhat(1:4)=xhat(1:4)/norm(xhat(1:4));
    xguzhi=xguzhi+0.5*dt*ou_w*[xguzhi(1) xguzhi(2) xguzhi(3) xguzhi(4)]';
    xguzhi=xguzhi/norm(xguzhi);
    F = [0 w_start(3)+xdotNoise(1)*randn+b3 -(w_start(2)+xdotNoise(1)*randn+b2) -0.5 0 0
        -(w_start(3)+xdotNoise(1)*randn+b3) 0 w_start(1)+xdotNoise(1)*randn+b1 0 -0.5 0
        w_start(2)+xdotNoise(1)*randn+b2 -(w_start(1)+xdotNoise(1)*randn+b1) 0 0 0 -0.5
        0 0 0 0 0 0
        0 0 0 0 0 0
        0 0 0 0 0 0];
    fai_transmat=eye(6)+F*dt+(dt^2/2)*F*F;
    x_predict=fai_transmat*x;
    G=[-0.5 0 0 0 0 0
        0 -0.5 0 0 0 0
        0 0 -0.5 0 0 0
        0 0 0 1 0 0
        0 0 0 0 1 0
        0 0 0 0 0 1];
    tao_mat=(eye(6)*dt+(dt^2/2)*F+(dt^2/6)*F*F)*G;
    P=fai_transmat*P*fai_transmat'+tao_mat*Q*tao_mat';
    H = [1 0 0 0 0 0
        0 1 0 0 0 0
        0 0 1 0 0 0];
    obver_predict=H*x_predict;
    xmeasure=[xtrue(2)+MeasNoise*randn xtrue(3)+MeasNoise*randn xtrue(4)+MeasNoise*randn]';
    z=[xmeasure(1)-xguzhi(2);xmeasure(2)-xguzhi(3);xmeasure(3)-xguzhi(4)];
    K=P*H'*(H*P*H'+R)^-1;
    x=K*z; 
    P=(eye(6)-K*H)*P*(eye(6)-K*H)'+K*R*K';
    Q_mul=[xhat(1) -xhat(2) -xhat(3) -xhat(4)
        xhat(2) xhat(1) -xhat(4) xhat(3)
        xhat(3) xhat(4) xhat(1) -xhat(2)
        xhat(4) -xhat(3) xhat(2) xhat(1)];
    xhat(1:4,1)=Q_mul*[sqrt(1-x(1)^2-x(2)^2-x(3)^2) x(1) x(2) x(3)]';
    xhat(1:4)=xhat(1:4)/norm(xhat(1:4));
    xhat(5:7,1)=xhat(5:7,1)+[x(4) x(5) x(6)]';
end

%xhatArray(1:4,:)=xhatArray(1:4,:)+0.0001*randn(4,361);
figure;
plot(tArray,xguzhiArray(1,:),'r:')
grid on;
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Time (Seconds)'); ylabel('q0');
figure;
plot(tArray,xguzhiArray(2,:),'r:')
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

figure;
plot(tArray, trPArray); title('Trace(P)', 'FontSize', 12);
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('Seconds');

figure
plot(xguzhiArray(end,:))
hold on;
plot(xhatArray(4,:))
hold on;
plot(xmeasureArray(end,:))
hold on;
plot(xtrueArray(end,:))
