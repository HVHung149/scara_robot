function tich=tich_co_huong(vector1,vector2)
tich=zeros(3,1);
tich(1)=-vector1(3)*vector2(2)+vector1(2)*vector2(3);
tich(2)=-vector1(3)*vector2(1)-vector1(1)*vector2(3);
tich(3)=-vector1(2)*vector2(1)+vector1(1)*vector2(2);
return;
end