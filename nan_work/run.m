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

global Xobs;
global Xobs0;
global vobs;

global T;
global l;
global TTC;

%% ego-vehicle state
l = 2.65;
X0 = [0, 2.5, 0]';
Xtarg = [20, 7.5, 0]';
v = 1;

%% guest-vehicle state
Xobs0 = [10; 2.5; 0];
vobs = 0.5;

%% weight of each component in objective function
w1 = .5;%length
w2 = [1, 10, 20];%last state distance 
w3 = 1;%smoothness
w4 = 0;%-0.5;%safety

%% receding time horizon setting
T = 20;
N = 100;

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
nonlcon = [];%@circlecon;
x0 = rand(1, N);
options = optimoptions('fmincon','Algorithm','sqp','MaxIterations',10000);
u = fmincon(@cost_fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

%% figure
figure(1)
plot([X0(1) X(1,:) Xtarg(1)],[X0(2) X(2,:) Xtarg(2)],'r-');
hold on
plot(Xobs(1,:),Xobs(2,:),'b-');
hold on
plot(X0(1),X0(2),'r*');
plot(Xtarg(1),Xtarg(2),'b*');
xlabel('x (m)');
ylabel('y (m)');
grid on

figure(2)
% vector1 = X - Xobs;
% dis = sqrt(vector1(1,:).^2+vector1(2,:).^2);
vector1 = Xobs - X;
for i =1:N
    v1 = [v*cos(X(3,i)), v*sin(X(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    v2 = [vobs*cos(Xobs(3,i)), vobs*sin(Xobs(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    TTC(i) = min((norm(vector1(1:2,i)))/(v1-v2),100);
    if (TTC(i)<0)
        TTC(i) = 100;
    end
 end
plot(1:N,TTC);
xlabel('time');
ylabel('distance to obstacle');
grid on


figure(3);
fontsize = 18;
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
    h1 = plot3(x1, y1, ones(1,5)*i/10, 'r-');
    hold on
    [x2, y2] = drawretangle(Xobs(:,i));
    h2 = plot3(x2, y2,ones(1,5)*i/10,'b-');
    hold on
end
plot3(X0(1),X0(2),0,'r*');
plot3(Xtarg(1),Xtarg(2),T,'b*');
xlabel('x (m)','fontsize',fontsize);
ylabel('y (m)','fontsize',fontsize);
zlabel('t (s)');
xlim([xlowerlim xupperlim]);
ylim([ylowerlim yupperlim]);
legend([h1 h2],'ego-vehicle','guest-vehicle')
set(gca,'FontSize',fontsize);
grid on;
    



    

