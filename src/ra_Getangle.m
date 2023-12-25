function [roll,pitch,yaw] = ra_Getangle(xAcce,yAcce,zAcce,xGyro,yGyro,zGyro)
%% b-frame: NED
%Gyro：rad/s
wieb=([xGyro,yGyro,zGyro])';
%Acce: m/s^2 [gb=-mean(f)]
gb=-([xAcce,yAcce,zAcce])';
%get vecs->mat
wg=gb/norm(gb);
ww=cross(gb,wieb)/norm(cross(gb,wieb));
wgw=cross(cross(gb,wieb),gb)/norm(cross(cross(gb,wieb),gb));
w=[wg,ww,wgw];

%% n-frame: NED
global g0 WIE lat lon;
gn=[0;0;g0];
wien=[WIE*cos(lat);0;-WIE*sin(lat)];
%get vecs->mat
vg=gn/norm(gn);
vw=cross(gn,wien)/norm(cross(gn,wien));
vgw=cross(cross(gn,wien),gn)/norm(cross(cross(gn,wien),gn));
v=[vg,vw,vgw];

%% get C:b->n and the EulerAngle Group
C_bn=v*w';

roll=atan2(C_bn(3,2),C_bn(3,3));
pitch=atan(-C_bn(3,1)/sqrt(C_bn(3,2)^2+C_bn(3,3)^2));
yaw=atan2(C_bn(2,1),C_bn(1,1));

end

