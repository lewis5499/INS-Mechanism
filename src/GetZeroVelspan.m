clc;clf;close;clear;
load ..\dataset\A15raw.mat;
cd ..\src\
global Plot2d Plot3d subPlot2 subPlot3;

% data=[];
% fileID = fopen('.\02 A15\11_31_52_CHA3_IMU.txt', 'r');
% tline = fgetl(fileID);
% while ischar(tline)
%     lineData =cell2mat(textscan(tline,'%f'))';  
%     data = [data; lineData];
%     tline = fgetl(fileID);
% end
% fclose(fileID);

% acc:
subPlot3(data(:,1),data(:,5),data(:,6),data(:,7))
% gyro:
subPlot3(data(:,1),data(:,2),data(:,3),data(:,4))
nn=zeros(362356,1);
for i=1:362356
    nn(i)=norm(data(i,5:7));
end
figure;
plot(data(:,1),nn,'Color',[0.88, 0.4, 0.4],LineWidth=1);