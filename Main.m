clear
close all

robot;
robot = r3_robot;

Vmax = [100 100 100 100 100 100]*pi/180;
a_max = [300 300 300 300 300 300]*pi/180;
% Vmax = [1 1 1 1 1 1];
% a_max = [1 1 1 1 1 1];
Vmax_c = 1;
a_max_c = 1;
dt = 0.01;


q1  = [ 0 -pi/2 pi/4 0 pi/2 0];
p = [0.4, 0, 0.45];
T=eye(4);
T(1:3,4) = p;
q2 = IK(T,robot);
% q2  = [ 0 pi/2 -pi/4 0 pi 0];
[V1 , t1] = calc_ptp(q1,q2,Vmax,a_max);

T1 = FK(q2,robot);
p1 = T(1:3,4);
p2 = [0.4, 0, -0.35 ];
[V2 , t2] = calc_LIN(p1',p2,Vmax_c,a_max_c,robot);

p3 = [0.4, 0.1, 0];
[V3, t3] = calc_ARC(p2, p3, p1',Vmax_c,a_max_c,robot);

T=eye(4);
T(1:3,4) = p1;
q3 = IK(T,robot);
q4 = [ pi/4 -pi/3 pi/6 pi/10 pi/2 0];
[V4 , t4] = calc_ptp(q3,q4,Vmax,a_max);

q5 = [ 0 0 0 0 0 0];

[V5 , t5] = calc_ptp(q4,q5,Vmax,a_max);

[V, t]= junction_6dof(V1,t1,V2,t2);
[V, t]= junction_6dof(V,t,V3,t3);
[V, t]= junction_6dof(V,t,V4,t4);
[V, t]= junction_6dof(V,t,V5,t5);




% V = [V1; V2; V3; V4; V5];
% t = [t1, t2+max(t1), t3+max(t2+max(t1)), t4+max(t3+max(t2+max(t1))), ...
%     t5+max(t4+max(t3+max(t2+max(t1))))];
% % 
% T = eye(4);
% T(1:3,4)= p2';
% q = IK(T,robot);
q=q1;
i=1;
hold on
for ti = t
   
   T = FK(q,robot);
%    plot3(T(1,4), T(2,4), T(3,4),'o')
   p(i,:) = T(1:3,4);
   q =q+V(i,:)*dt;
   i=i+1;
end

plot3(p(:,1),p(:,2),p(:,3));
title('Trajectory with juncion')

% plot joint velocity
figure();
plot(t,V(:,2));
title('Velocity with juncion')

V = [V1; V2; V3; V4; V5];
t = [t1, t2+max(t1), t3+max(t2+max(t1)), t4+max(t3+max(t2+max(t1))), ...
    t5+max(t4+max(t3+max(t2+max(t1))))];

figure();
q=q1;
i=1;
hold on
for ti = t
   
   T = FK(q,robot);
%    plot3(T(1,4), T(2,4), T(3,4),'o')
   p(i,:) = T(1:3,4);
   q =q+V(i,:)*dt;
   i=i+1;
end

plot3(p(:,1),p(:,2),p(:,3));
title('Trajectory without juncion')

% plot joint velocity
figure();
plot(t,V(:,2));
title('Velocity without juncion')


% plot3(p1(1),p1(2),p1(3),'*');
% plot3(p2(1),p2(2),p2(3),'*');
% plot3(p3(1),p3(2),p3(3),'*');
