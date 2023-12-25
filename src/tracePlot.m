%clc;clf;close;clear;
%load data1.mat
%load dataA15withZeroVelCorr.mat
load ..\dataset\data_demo.mat
cd ..\src\
%load data_demo_ref.mat

global Plot2d Plot3d subPlot2 subPlot3;
Plot2d = @myPlot2d;
Plot3d = @myPlot3d;
subPlot2 = @mysubPlot2;
subPlot3 = @mysubPlot3;

[n,e] = blh2ne(lat,lon,height);
Plot2d(e,n)
Plot2d(lon,lat)
Plot3d(lon,lat,height)

function []=myPlot2d(lon,lat)
figure;
set(gcf,'Position',[0 0 1000 1500])
plot(lon,lat,LineWidth=1);
box on
grid on 
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
legend1=legend('$\bf{Trace}$','interpreter','latex','FontSize',10.5); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
xlabel('$\bf{East(m)}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{North(m)}$','interpreter','latex','FontSize', 17)  
title({'$\bf{Nav-Trajectory(2D)}$'}, 'interpreter','latex','FontSize', 19);
end

function []=myPlot3d(lon,lat,height)
figure;
set(gcf,'Position',[0 0 1000 1500])
plot3(lon,lat,height,'Color',[0.88, 0.4, 0.4],"LineWidth",1);
grid on
title({'$\bf{Nav-Trajectory(3D)}$'}, 'interpreter','latex','FontSize', 19);
xlabel('$\bf{lon(deg)}$','interpreter','latex','FontSize', 17)
ylabel('$\bf{lat(deg)}$','interpreter','latex','FontSize', 17)  
zlabel('$\bf{height(m)}$','interpreter','latex','FontSize', 17)
legend1=legend('$\bf{Trace}$','interpreter','latex','FontSize',10.5,'Location','northeast'); 
set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
end

function []=mysubPlot2(t,value1,value2)
figure;
set(gcf,'Position',[0 0 1000 1500])

subplot(2,1,1)
plot(t,value1,'Color',[0.88, 0.4, 0.4],LineWidth=1);
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
%legend1=legend('$\bf{Val1}$','interpreter','latex','FontSize',10.5); 
%set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid on 

subplot(2,1,2)
plot(t,value2,'Color',[0.9290 0.6940 0.1250],LineWidth=1);
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
%legend1=legend('$\bf{Val2}$','interpreter','latex','FontSize',10.5); 
%set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
box on
grid on 

%xlabel('$\bf{time(sec)}$','interpreter','latex','FontSize', 17)
%ylabel('$\bf{value}$','interpreter','latex','FontSize', 17)  
%title({'$\bf{Nav-Trajectory(2D)}$'}, 'interpreter','latex','FontSize', 19);
end

function []=mysubPlot3(t,value1,value2,value3)
figure; 
set(gcf,'Position',[0 0 1000 1500])

subplot(3,1,1)
plot(t,value1,'Color',[0.62 0.49 0.31],LineWidth=1);hold on
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
%legend1=legend('$\bf{rollRes}$','interpreter','latex','FontSize',10.5); 
%set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
%ylabel('$\bf{dRoll(deg)}$','interpreter','latex','FontSize', 17)  
%title({'$\bf{Nav-RawGyroOutput(d\theta-to-\omega)}$'}, 'interpreter','latex','FontSize', 18);
%title({'$\bf{Nav-Residual(roll-pitch-yaw)}$'}, 'interpreter','latex','FontSize', 18);
box on
grid on 

subplot(3,1,2)
plot(t,value2,'Color',[0.4660 0.6740 0.1880],LineWidth=1);hold on
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
%legend1=legend('$\bf{pitchRes}$','interpreter','latex','FontSize',10.5); 
%set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
%ylabel('$\bf{dPitch(deg)}$','interpreter','latex','FontSize', 17)  
box on
grid on 

subplot(3,1,3)
plot(t,value3,'Color',[0.8500 0.3250 0.0980],LineWidth=1);hold on
set(gca,'linewidth',1.2,'fontsize',14,'fontname','Times','FontWeight','bold')
%legend1=legend('$\bf{yawRes}$','interpreter','latex','FontSize',10.5); 
%set(legend1,'LineWidth',1,'Interpreter','latex','FontSize',10.5);
%ylabel('$\bf{dYaw(deg)}$','interpreter','latex','FontSize', 17)  
%xlabel('$\bf{time(sec)}$','interpreter','latex','FontSize', 17)
box on
grid on 
hold off
end



