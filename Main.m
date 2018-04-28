clear
close all

robot;
robot = r3_robot;

Vmax = [100 100 100 100 100 100]*pi/180;
a_max = [300 300 300 300 300 300]*pi/180;
Vmax_c = 1;
a_max_c = 1;
dt = 0.01;


q1  = [ 0 pi/2 0 pi/2 pi 0];
q2  = [ pi/4 0 pi/4 0 0 0];

p1 = [1, 0.3, 0.4];
p2 = [0, 0, 1];


[V , t] = calc_ptp(q1,q2,Vmax,a_max);

[V , t] = calc_LIN(p1,p2,Vmax_c,a_max_c);

T = eye(4);
T(1:3,4)= p1';
q = IK(T,robot);
i=1;
for ti = t
   J = Jac_Agilus(q,robot); 
   dq(i,:) = J * [V(:,i)' 0 0 0]';
   
   q =q+dq(i,:)*dt;
   i=i+1;
end

% plot(t,dq(:,6));


