clear
close all

robot;
robot = r3_robot;

Vmax = [100 100 100 100 100 100]*pi/180;
a_max = [300 300 300 300 300 300]*pi/180;
Vmax_c = 1;
a_max_c = 1;
dt = 0.01;


q1  = [ 0 -pi/2 pi/4 0 pi/2 0];
p1 = [0.4, 0, 0.45];
T=eye(4);
T(1:3,4) = p1;
q2 = IK(T,robot);
% q2  = [ 0 pi/2 -pi/4 0 pi 0];

% p1 = [0.4, 0, 0.45];
p2 = [0.6, 0.1, 0];
p3 = [0.4, 0, -0.35 ];


% Jac_Agilus(q1,robot)
% syms q1 q2 q3 q4 q5 q6 
% 
% T = FK([q1 q2 q3 q4 q5 q6],robot);
% 
% X = T(1,4);
% Y = T(2,4);
% Z = T(3,4);
% 
% J(q1,q2,q3,q4,q5,q6) = ... 
%     [diff(X,'q1') diff(X,'q2') diff(X,'q3') diff(X,'q4') diff(X,'q5') diff(X,'q6');...
%     diff(Y,'q1') diff(Y,'q2') diff(Y,'q3') diff(Y,'q4') diff(Y,'q5') diff(Y,'q6');...
%     diff(Z,'q1') diff(Z,'q2') diff(Z,'q3') diff(Z,'q4') diff(Z,'q5') diff(Z,'q6');];
% 
% vpa(J(0, -pi/2, pi/4, 0, pi/2, 0),3)



% % [V , t] = calc_ptp(q1,q2,Vmax,a_max);
% 
% [V , t] = calc_LIN(p1,p2,Vmax_c,a_max_c,robot);
% 
[V, t] = calc_ARC(p1,p2,p3,Vmax_c,a_max_c,robot);
% 
T = eye(4);
T(1:3,4)= p1';
q = IK(T,robot);
i=1;
hold on
for ti = t
   
   T = FK(q,robot);
   plot3(T(1,4), T(2,4), T(3,4),'o')
   q =q+V(i,:)*dt;
   i=i+1;
end

% plot(t,V(:,1));
plot3(p1(1),p1(2),p1(3),'*');
plot3(p2(1),p2(2),p2(3),'*');
plot3(p3(1),p3(2),p3(3),'*');
