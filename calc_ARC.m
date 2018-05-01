function [dq, t] = calc_ARC(p1,p2,p3,Vmax_c,a_max_c,robot)


dt = 0.01;

D = [p1;p2;p3];
D1 = D;
D1(1:3,1) = [1 1 1]';
D2 = D;
D2(1:3,2) = [1 1 1]';
D3 = D;
D3(1:3,3) = [1 1 1]';

Z = [det(D1) det(D2) det(D3)] / det(D);
X = p2 - p1;
Y = cross(Z,X);

x1 = 0; y1=0;
x2 = sqrt(sum(X.^2)); y2 = 0;

Z = Z / sqrt(sum(Z.^2));
X = X / sqrt(sum(X.^2));
Y = Y / sqrt(sum(Y.^2));

x3_ = cross(p3-p1,Y);

if ((p3-p1)*Y' > 0)
    x3 = -sqrt(sum(x3_.^2));
else
    x3 = sqrt(sum(x3_.^2));
end
y3_ = cross(p3-p1,X);

if ((p3-p1)*X' > 0)
    y3 = -sqrt(sum(y3_.^2));
else
    y3 = sqrt(sum(y3_.^2));
end


x0 = -0.5 * (y2 * (x3 * x3 + y3 * y3) + y3 * (0 - x2 * x2 - y2 * y2)) / (x2 * y3 + x3 * (-y2));
y0 = 0.5 * (x2 * (x3 * x3 + y3 * y3) + x3 * (0 - x2 * x2 - y2 * y2)) / (x2 * y3 + x3 * (-y2));

R = sqrt( x0 * x0 + y0 * y0); 

p0 = p1 + X*x0 + Y*y0;

phi1 = atan2((y1-y0) / R, (x1-x0) / R);
phi2 = atan2((y3 - y0) / R, (x3 - x0) / R);

L = abs(R * (phi2 - phi1));

ta = Vmax_c/a_max_c;
ta = ceil(ta/dt)*dt;
tb = L/Vmax_c - ta;
tb = ceil(tb/dt)*dt + ta;
if ((tb-ta)>0)
    %     trap
    t=0:dt:(ta+tb)+dt;
    v = L/tb;
    a = v/ta;
    
    n = int32(ta/dt);
    n2 = int32((tb-ta)/dt);
    a_j(1 : n) = a;
    a_j(n+1 : (n+1+n2)) = 0;
    a_j(n+2+n2 : 2*n+n2+2) = -a;
    
else
    %     triangle
    t=0:dt:2*ta;
    
    n = int32(ta/dt);
    a_j(1:n) = L/(ta^2);
    a_j(n+1:2*n+1) = -L/(ta^2);
end

i=1;
for ti = t
    if (ti~=0)
        V_m(i) =V_m(i-1)+ a_j(i-1)*dt;
    else
        V_m(i) = 0;
    end
    i=i+1;
end
n=i-1;

dphi = (phi2 - phi1)/n;

phi = phi1;
p_old = p1;
% hold on
for i=1:n
    phi = phi + sign(dphi)*V_m(i)*dt/R;
    x = x0 + R * cos(phi);
    y = y0 + R * sin(phi); 
    
    p = p1 + X*x + Y*y;
%     plot3(p(1),p(2),p(3),'.');
%     V_vec(i,:) = cross(p-p0, Z);
%     V_vec(i,:) = V_vec(i,:)/ sqrt(sum(V_vec(i,:).^2));
%     plot3([p(1); p(1)+V_vec(i,1)], [p(2); p(2)+V_vec(i,2)], [p(3); p(3)+V_vec(i,3)])
    V(:,i) = (p - p_old)/dt;
    p_old = p;
end

% plot3(p1(1),p1(2),p1(3),'*');
% plot3(p2(1),p2(2),p2(3),'*');
% plot3(p3(1),p3(2),p3(3),'*');

% plot(V(1,:)) 
p=p1;
T = eye(4);
T(1:3,4)= p1';
q = IK(T,robot);
i=1;
for ti = t
   J = Jac_Agilus(q,robot); 
   dq(i,:) = J \ [V(:,i)' 0 0 0]';
   
   q =q+dq(i,:)*dt;
   
   
   p = p + V(:,i)'*dt;
%    plot3(p(1),p(2),p(3),'o');
   i=i+1;
end
 


end