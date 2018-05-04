function [V, t] = junction_6dof(V1,t1,V2,t2)

steps = 5;

n1 = size(t1,2);
n2 = size(t2,2);
dt=0.01;

t_f = dt*(n1+n2-6);
t = 0:dt:t_f;

V = zeros(n2+n1-5,6);
V(1:n1,:) = V1;

for i = 1 :steps
    V(n1-i+1,:) = V1(n1-i+1,:) + V2(steps+1-i,:);
end
V(n1+1 : n2+n1-steps,:) = V2(steps+1:n2,:); 



end