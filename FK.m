function T = FK(q,robot,n)

del_q3=atan2(robot.Links(5).length,robot.Links(4).length);
d3_=sqrt(robot.Links(5).length^2+robot.Links(4).length^2);
T1=Tz(robot.Links(1).length)*Rz(q(1)+robot.Joints(1).error)*Tx(robot.Links(2).length);
T2=Ry(q(2)+robot.Joints(2).error)*Tx(robot.Links(3).length);
T3=Ry(q(3)+robot.Joints(3).error + del_q3)*Tx(d3_)*Ry(-del_q3);
T4=Rx(q(4)+robot.Joints(4).error);
T5=Ry(q(5)+robot.Joints(5).error);
T6=Rx(q(6)+robot.Joints(6).error)*Tx(robot.Links(6).length);

T = T1*T2*T3*T4*T5*T6;

end