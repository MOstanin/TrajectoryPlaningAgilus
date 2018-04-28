function [V , t] = calc_ptp(q1,q2,Vmax,a_max)

dt = 0.01; %c

n =  size(q1,2);

for i = 1:n
    ta(i) = Vmax(i)/a_max(i);
    ta(i) = ceil(ta(i)/dt)*dt;
    tb(i) = abs((q2(i)-q1(i)))/Vmax(i);
end
tm_a =  max(ta);
for i = 1:n
    if (tb(i)>0)
        tb(i) = tb(i) - tm_a;
    end
end
if (min(tb)<0)
%     triangle
    tm_b=0;
else
%     trapze
tm_b =  max(tb);
tm_b = ceil(tm_b/dt)*dt+tm_a;
end

for i = 1:n
    a_j = calc_a(q2(i)-q1(i), tm_a, tm_b,dt);
    a(i,:) = a_j;
end
if (tm_b ==0)
    t = 0:dt:2*tm_a;
else
    t = 0:dt:(tm_a+tm_b)+dt;
end
for j=1:n
i=1;
for ti = t
    if (ti~=0)
        V(j,i) =V(j,i-1)+ a(j,i-1)*dt;
    else
        V(j,i) = 0;
    end
    i=i+1;
end
end


end

function a_j = calc_a(dq, tm_a, tm_b, dt)
    
    if tm_b ==0
        n = int32(tm_a/dt);
        a_j(1:n) = dq/(tm_a^2);
        a_j(n+1:2*n+1) = -dq/(tm_a^2);
    else
        v = dq/tm_b;
        a = v/tm_a;
        
        n = int32(tm_a/dt);
        n2 = int32((tm_b-tm_a)/dt);
        a_j(1:n) = a;
        a_j(n+1:(n+1+n2)) = 0;
        a_j(n+2+n2:2*n+n2+2) = -a;
        
    end
end




