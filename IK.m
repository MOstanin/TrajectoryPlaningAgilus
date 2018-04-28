function q = IK(Tgoal,robot,n)

del_q3=atan2(robot.Links(5).length,robot.Links(4).length);
d3_=sqrt(robot.Links(5).length^2+robot.Links(4).length^2);

T0= Tgoal*inv(Tx(robot.Links(6).length));
T123ans=T0;
q1_=atan2(T123ans(2,4),T123ans(1,4));
a=sqrt(T123ans(2,4)^2+T123ans(1,4)^2)-robot.Links(2).length;
b=T123ans(3,4)-robot.Links(1).length;
c3=(a^2+b^2-robot.Links(3).length^2-d3_^2)/...
    (2*robot.Links(3).length*d3_);
s3=sqrt(1-c3^2);
q3_=atan2(s3,c3);
q3= q3_- del_q3;
q2_=-atan2(d3_*sin(q3_),(robot.Links(3).length+d3_*cos(q3_)))+atan2(a,b);
q2 = q2_-pi/2;
T1=Tz(robot.Links(1).length)*Rz(q1_)*Tx(robot.Links(2).length);
T2=Ry(q2)*Tx(robot.Links(3).length);
T3=Ry(q3_)*Tx(d3_)*Ry(-del_q3);

T456ans = inv(T1*T2*T3)*T0;

if  abs(T456ans(1,1))~=1 
    q4_=atan2(T456ans(2,1),-T456ans(3,1));
    q6_=atan2(T456ans(1,2),T456ans(1,3));
    if T456ans(1,3)~=0
        q5_=atan2(T456ans(1,3)/cos(q6_),T456ans(1,1));
    else
        q5_=atan2(T456ans(1,2)/sin(q6_),T456ans(1,1));
    end
else
    disp('Singulatuty in wirst')
    q5_= acos(T456ans(1,1));
    q4_ = 0;
    q6_ = 0;
end
q = [q1_ q2 q3 q4_ q5_ q6_];
end