clc;clf;close;clear;
load ..\dataset\diffdata_demoFINALcar.mat
cd ..\src\
global Plot2d Plot3d subPlot2 subPlot3;

INSresultsref.yaw(INSresultsref.yaw>180)=INSresultsref.yaw(INSresultsref.yaw>180)-2*180;

time=INSresultsdemo.t;
dlatlon=[INSresultsdemo.lat-INSresultsref.lat ...
         INSresultsdemo.lon-INSresultsref.lon];
% dlatlon=[INSresultsdemo.lat...
%          INSresultsdemo.lon];
dheight=INSresultsdemo.height-INSresultsref.height;
% dheight=INSresultsdemo.height;
dvel=[INSresultsdemo.vn-INSresultsref.vn ...
      INSresultsdemo.ve-INSresultsref.ve ...
      INSresultsdemo.vd-INSresultsref.vd];
% dvel=[INSresultsdemo.vn ...
%       INSresultsdemo.ve ...
%       INSresultsdemo.vd];
datt=[INSresultsdemo.roll-INSresultsref.roll ...
      INSresultsdemo.pitch-INSresultsref.pitch ...
      INSresultsdemo.yaw-INSresultsref.yaw];
% datt=[INSresultsdemo.roll ...
%       INSresultsdemo.pitch ...
%       INSresultsdemo.yaw];

subPlot3(time,dlatlon(:,1),dlatlon(:,2),dheight);
subPlot3(time,dvel(:,1),dvel(:,2),dvel(:,3))
subPlot3(time,datt(:,1),datt(:,2),datt(:,3))

maxdlatlon=max(max(abs(dlatlon)));
maxdheight=max(max(abs(dheight)));
maxdvel=max(max(abs(dvel)));
maxdatt=max(max(abs(datt)));