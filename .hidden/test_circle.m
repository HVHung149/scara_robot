r=400   %thoa dieu kien 
xA=650
yA=0
A=[xA yA]'
xB=200
yB=400
B=[xB yB]'

AB=sqrt((A(1)-B(1))^2+(A(2)-B(2))^2)

phi=acos((2*r^2-AB^2)/(2*r^2))*180/pi

L=2*pi*r*(phi/360)