% created by Hengzhen Liu, Oct. 12nd, 2023
%% init
clc;clear;
global g0 e R WIE lat lon;
% earth-params
WIE=7.292115e-5;
e=0.0818191908426;
R=6378137.0;
g0=9.7936174;
lat=30.52780368;
lon=114.35579096;
%% load OBS
load ..\dataset\A15raw.mat;
cd ..\src\

xGyro=data(1:60778,2);
yGyro=data(1:60778,3);
zGyro=data(1:60778,4);
xAcce=data(1:60778,5);
yAcce=data(1:60778,6);
zAcce=data(1:60778,7);
%% get mean angle 
[roll,pitch,yaw]=ra_Getangle(mean(xAcce),mean(yAcce),mean(zAcce), ...
                             mean(xGyro),mean(yGyro),mean(zGyro));
disp('mean value in the whole period:')
disp('roll = '),disp(rad2deg(roll));
disp('pitch = '),disp(rad2deg(pitch));
disp('yaw = '),disp(rad2deg(yaw));

















