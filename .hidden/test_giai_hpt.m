close all
clear
clc
syms t m
flag=0;
choose_C1=0;
choose_C2=0;
r=350
A=[650 0]
B=[200 400]
% f1 =(A(1)-t)^2+(A(2)-m)^2-r^2;
% f2 =(B(1)-t)^2+(B(2)-m)^2-r^2;
% nghiem= solve(f1,f2,t,m)
% C1=[double(nghiem.t(1)) double(nghiem.m(1))]


% C2=[double(nghiem.t(2)) double(nghiem.m(2))]

AB=sqrt((A(1)-B(1))^2+(A(2)-B(2))^2);
M =[(A(1)+B(1))/2 (A(2)+B(2))/2];
C =getCenter(A,B,r);
if(C(1,1)<M(1))
    center_x=C(1,1)
    center_y=C(1,2)
else
    center_x=C(2,1)
    center_y=C(2,2)
end
%set qmax
% if(handles.rdo_clockwise.Value== true)
if(flag==0)
       if(B(2)>0)
       qmax =GetQmax(r,AB,false)
   else
       qmax =GetQmax(r,AB,true)
   end
else
       if(B(2)>0)
       qmax =GetQmax(r,AB,true)
   else
       qmax =GetQmax(r,AB,false)
   end
end
% end
% if(handles.rdo_counter_clockwise.Value== true)

% end
amax=10;
vmax=10;
length=200;
t(2) = (2*vmax/amax);  %tc
t(1)=   t(2)/2;
t(5) = qmax/vmax + t(2);
t(3) = t(5) - t(2);
t(4) = t(5) - t(2)/2;
t_unit = t(5)/length;
x=zeros(length,1);
for i = 1:length
    if i<length
    x(i+1) = x(i) +t_unit;
    end
    if x(i) < t(1)
         y3(i) = (1/3)*(amax/t(2))*x(i)^3;
    elseif x(i) < t(2)
         y3(i) = -(1/3)*(amax/t(2))*x(i)^3 + amax*x(i)^2 - vmax*x(i)-...
                (1/6)*(amax*t(2)^2)+vmax*(t(2)/2);
    elseif x(i)< t(3)
         y3(i) =amax*t(2)*x(i)/2-amax*t(2)^2/4;
    elseif x(i)< t(4)
        y3(i) = -amax*(x(i)-t(5)+t(2))^3/3/t(2) + amax*t(2)*x(i)/2 -amax*t(2)^2/4;
    elseif x(i)< t(5)
        y3(i) = amax*(x(i)-t(5))^3/3/t(2)+amax*t(2)*(t(5)-t(2))/2;
    end
end
if(center_y>=0)
    phi= -acos((650-C(1,1))/r)
else
    phi= acos((650-C(1,1))/r)
end

%phi=-(pi-acos((-650+C2(1))/r))
phi_dree=phi*180/pi
qx = qx_circle(-y3,r);
qy = qy_circle(-y3,r);
DH= [cos(phi) -sin(phi) 0 C(1,1);...
    sin(phi) cos(phi) 0 C(1,2);...
    0 0 1 0;...
    0 0 0 1];

P=zeros(length,4);
% P=[C1(1) C1(2) 0 0]'+DH*[qx(i) qy(i) 0 1]'
%comment
distance=zeros(length,1);
for i = 1:length
    if i<length
    x(i+1) = x(i) +t_unit;
    end
   P(i,:)=(DH*[qx(i) qy(i) 0 1]')';
   distance(i,1)=sqrt(P(i,1)^2+P(i,2)^2);
end
double(P)
double(P(1,1))
double(P(1,2))
double(P(length,1))
double(P(length,2))
if(P(1,1)==650 && abs(P(1,2))>=0 && abs(P(length,1)-B(1))<=5 && abs(P(length,2)-B(2))<=5)
    choose_C1=1;
    disp('accept C1');
else
    disp('dont accept C1');
end

% double(distance)
% plot(P(:,1),P(:,2));
% hold on
% flag=0;
% if(flag==1)
%        if(B(2)>0)
%        qmax =GetQmax(r,AB,false)
%    else
%        qmax =GetQmax(r,AB,true)
%    end
% else
%         if(B(2)>0)
%        qmax =GetQmax(r,AB,true)
%    else
%        qmax =GetQmax(r,AB,false)
%    end
% end
% t(2) = (2*vmax/amax);  %tc
% t(1)=   t(2)/2;
% t(5) = qmax/vmax + t(2);
% t(3) = t(5) - t(2);
% t(4) = t(5) - t(2)/2;
% t_unit = t(5)/length;
% x=zeros(length,1);
% for i = 1:length
%     if i<length
%     x(i+1) = x(i) +t_unit;
%     end
%     if x(i) < t(1)
%          y3(i) = (1/3)*(amax/t(2))*x(i)^3;
%     elseif x(i) < t(2)
%          y3(i) = -(1/3)*(amax/t(2))*x(i)^3 + amax*x(i)^2 - vmax*x(i)-...
%                 (1/6)*(amax*t(2)^2)+vmax*(t(2)/2);
%     elseif x(i)< t(3)
%          y3(i) =amax*t(2)*x(i)/2-amax*t(2)^2/4;
%     elseif x(i)< t(4)
%         y3(i) = -amax*(x(i)-t(5)+t(2))^3/3/t(2) + amax*t(2)*x(i)/2 -amax*t(2)^2/4;
%     elseif x(i)< t(5)
%         y3(i) = amax*(x(i)-t(5))^3/3/t(2)+amax*t(2)*(t(5)-t(2))/2;
%     end
% end
% phi=-acos((650-C1(1))/r)
% qx = qx_circle(y3,r);
% qy = qy_circle(y3,r);
% for i = 1:length
%     if i<length
%     x(i+1) = x(i) +t_unit;
%     end
%     P(i,:)=(DH*[qx(i) qy(i) 0 1]')';
%     distance(i,1)=sqrt(P(i,1)^2+P(i,2)^2);
% end
% double(P)
% double(distance)
% figure
% plot(P(:,1),P(:,2));
% hold off
% % PA= DH*[  -54.4357 -396.2786  0 1]'
