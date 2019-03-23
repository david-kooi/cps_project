clc
clear all
close all

%% global setting
global X0;
global Xtarg;
global X;
global v;

global w1;
global w2;
global w3;
global w4;
global w5;
global w6;

global Xobs;
global Xobs0;
global vobs;

global T;
global l;
global TTC;

mark = 2;
weight_mark = 0;
%% ego-vehicle state
l = 2.65;
X0 = [0, 2.5, 0]';
Xtarg = [20, 7.5, 0]';
v = 1;

%% guest-vehicle state
Xobs0 = [15; 2.5; 0];
vobs = 0.5;

%% weight of each component in objective function
w1 = .5;%length
w2 = [1, 10, 30];%last state distance 
w3 = 1;%smoothness
w4 = 0;%-0.5;%safety
w5 = 1;
w6 = 5;
%% receding time horizon setting
T = 20;
N = 50;

%% memory value
global mem_TTC;
mem_TTC = [];

%% main
A = [];
b = [];
Aeq = [];
beq = [];
lb = -0.5 * ones(1, N);
ub = 0.5 * ones(1, N);
nonlcon = @circlecon;
%nonlcon = [];
x0 = zeros(1, N);
options = optimoptions('fmincon','Algorithm','sqp','MaxIterations',10000);
u = fmincon(@cost_fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

%% figure
fontsize = 28;

figure(1)
%subplot(2,2,1)
% plot([X0(1) X(1,:) Xtarg(1)],[X0(2) X(2,:) Xtarg(2)],'r*');
% hold on
% plot(Xobs(1,:),Xobs(2,:),'b*');
% hold on
% plot(X0(1),X0(2),'r*');
% plot(Xtarg(1),Xtarg(2),'b*');
% xlabel('x (m)','fontsize',fontsize);
% ylabel('y (m)','fontsize',fontsize);
% set(gca,'FontSize',fontsize);
% grid on

xupperlim =25;
xlowerlim =-2;
yupperlim = 10;
ylowerlim = 0;
ratio = 50;
width= (xupperlim - xlowerlim)*ratio;
height= (yupperlim - ylowerlim)*ratio;
left=200;
bottem=100;
set(gcf,'position',[left,bottem,width,height])
for i = 1:N
    [x1, y1] = drawretangle(X(:,i));
    h1 = plot(x1, y1, 'r-');
    hold on
    [x2, y2] = drawretangle(Xobs(:,i));
    h2 = plot(x2, y2,'b-');
    hold on
end
plot(X0(1),X0(2),'r*');
plot(Xtarg(1),Xtarg(2),'b*');
xlabel('x (m)','fontsize',fontsize);
ylabel('y (m)','fontsize',fontsize);
xlim([xlowerlim xupperlim]);
ylim([ylowerlim yupperlim]);
legend([h1 h2],'ego-vehicle','guest-vehicle')
set(gca,'FontSize',fontsize);
grid on;


% figure(1)
% subplot(2,2,2)
figure(2)
fontsize = 18;
% vector1 = X - Xobs;
% dis = sqrt(vector1(1,:).^2+vector1(2,:).^2);
vector1 = Xobs - X;
for i =1:N
    v1 = [v/cos(X(3,i))*cos(X(3,i)), v/cos(X(3,i))*sin(X(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    v2 = [vobs*cos(Xobs(3,i)), vobs*sin(Xobs(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    TTC(i) = min((norm(vector1(1:2,i)))/(v1-v2),100);
    if (TTC(i)<0)
        TTC(i) = 100;
    end
 end
plot(1:N,TTC);
xlabel('time (s)','fontsize',fontsize);
ylabel('Time to Collision (s)','fontsize',fontsize);
set(gca,'FontSize',fontsize);
grid on

% figure(1)
% subplot(2,2,3)
fontsize = 18;
figure(3)
stairs((1:N)/N*T,u,'b-')
%hold on
%stem((1:N-1)/N*T,diff(u),'b-')
xlabel('time (s)','fontsize',fontsize);
ylabel('Front wheel angle','fontsize',fontsize);
set(gca,'FontSize',fontsize);
grid on


% figure(1)
% subplot(2,2,4)
figure(4)
fontsize = 18;
h1 = plot((1:N)/N*T,X(1,2:N+1),'g*');
hold on
h2 = plot((1:N)/N*T,X(2,2:N+1),'b*');
hold on
h3 = plot((1:N)/N*T,X(3,2:N+1),'r*');
hold on
legend([h1 h2 h3],'X','Y','Heading')
set(gca,'FontSize',fontsize);
grid on
% 
% figure(2);
fontsize = 28;
figure(5)

xupperlim =25;
xlowerlim =-2;
yupperlim = 10;
ylowerlim = 0;
ratio = 50;
width= (xupperlim - xlowerlim)*ratio;
height= (yupperlim - ylowerlim)*ratio;
left=200;
bottem=100;
set(gcf,'position',[left,bottem,width,height])
for i = 1:N
    [x1, y1] = drawretangle(X(:,i));
    h1 = plot3(x1, y1, ones(1,5)*i/N*T, 'r-');
    hold on
    [x2, y2] = drawretangle(Xobs(:,i));
    h2 = plot3(x2, y2,ones(1,5)*i/N*T,'b-');
    hold on
end
plot3(X0(1),X0(2),0,'r*');
plot3(Xtarg(1),Xtarg(2),T,'b*');
xlabel('x (m)','fontsize',fontsize);
ylabel('y (m)','fontsize',fontsize);
zlabel('time (s)');
xlim([xlowerlim xupperlim]);
ylim([ylowerlim yupperlim]);
legend([h1 h2],'ego-vehicle','guest-vehicle')
set(gca,'FontSize',fontsize);
grid on;
    

saveas(1,['EX' mat2str(mark) '_weight_' mat2str(weight_mark) '_trajectory.jpg']);
saveas(2,['EX' mat2str(mark) '_weight_' mat2str(weight_mark) '_Time_to_Collision.jpg']);
saveas(3,['EX' mat2str(mark) '_weight_' mat2str(weight_mark) '_Front_Wheel_Angle.jpg']);
saveas(4,['EX' mat2str(mark) '_weight_' mat2str(weight_mark) '_state.jpg']);
saveas(5,['EX' mat2str(mark) '_weight_' mat2str(weight_mark) '_trajectory3D.jpg']);

    

