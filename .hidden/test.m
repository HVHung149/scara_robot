a=[400 250 0 0]';
b=inverse_kinematic(200,-400,-50,180,a)
alpha=[0 0 0 pi]';
d=[383.5 0 b(3) 0]';
theta=[b(1) b(2) 0 b(4)]';
P=forward_kinematic(theta,alpha,d,a)