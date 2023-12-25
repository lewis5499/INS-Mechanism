function [n,e] = blh2ne(lat,lon,height)
len=length(lat);
n=zeros(len,1);
e=zeros(len,1);

WGS84_WIE = 7.2921151467E-5;     %  /* 地球自转角速度*/
WGS84_F   = 0.0033528106647474805; %/* 扁率 */
WGS84_RA  = 6378137.0000000000;   % /* 长半轴a */
WGS84_RB  = 6356752.3142451793;   % /* 短半轴b */
WGS84_GM0 = 398600441800000.00;   % /* 地球引力常数 */
WGS84_E1  = 0.0066943799901413156;% /* 第一偏心率平方 */
WGS84_E2  = 0.0067394967422764341; %/* 第二偏心率平方 */

for i=1:len
    Rm=WGS84_RA*(1-WGS84_E1)/sqrt((1-WGS84_E1*(sin(deg2rad(lat(i)))^2))^3);
    Rn=WGS84_RA/sqrt(1-WGS84_E1*(sin(deg2rad(lat(i)))^2));
    n(i)=(deg2rad(lat(i))-deg2rad(lat(1)))*(Rm+height(i));
    e(i)=(deg2rad(lon(i))-deg2rad(lon(1)))*(Rn+height(i))*cos(deg2rad(lat(i)));
end

