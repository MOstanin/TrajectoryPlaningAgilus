function [dq , t] = calc_LIN(p1,p2,Vmax_c,a_max_c,robot)
dt=0.01;
V_vec = p2-p1;
L = sqrt(sum(V_vec.^2));

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
V_vec = V_vec/L;

for i =1:n
    V(:,i) = V_vec*V_m(i); 
end

T = eye(4);
T(1:3,4)= p1';
q = IK(T,robot);
i=1;
for ti = t
   J = Jac_Agilus(q,robot); 
   dq(i,:) = inv(J) * [V(:,i)' 0 0 0]';
   
   q =q+dq(i,:)*dt;
   i=i+1;
end



end