function J = Jac_Agilus(q,robot,tool)


Htx = [0 0 0 1;
    0 0 0 0; 
    0 0 0 0;
    0 0 0 0];
Hty = [0 0 0 0;
    0 0 0 1;
    0 0 0 0; 
    0 0 0 0];
Htz = [0 0 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0];
Hrx = [0 0 0 0;
    0 0 -1 0;
    0 1 0 0;
    0 0 0 0];
Hry = [0 0 1 0;
    0 0 0 0;
    -1 0 0 0;
    0 0 0 0];
Hrz = [0 -1 0 0;
    1 0 0 0;
    0 0 0 0; 
    0 0 0 0];

T = FK(q,robot);
R = T(1:3,1:3);

% del_q3=atan2(robot.Links(5).length,robot.Links(4).length);
% d3_=sqrt(robot.Links(5).length^2+robot.Links(4).length^2);

T1=Tz(robot.Links(1).length)*Rz(q(1)+robot.Joints(1).error)*Tx(robot.Links(2).length);
T2=Ry(q(2)+robot.Joints(2).error)*Tx(robot.Links(3).length);
T3 =Ry(q(3)+robot.Joints(3).error)*Tz(robot.Links(4).length)*Tx(robot.Links(5).length);
T4=Rx(q(4)+robot.Joints(4).error);
T5=Ry(q(5)+robot.Joints(5).error);
T6=Rx(q(6)+robot.Joints(6).error)*Tx(robot.Links(6).length);

T1Hr=Tz(robot.Links(1).length)*Rz(q(1)+robot.Joints(1).error)*Hrz*Tx(robot.Links(2).length);
T2Hr=Ry(q(2)+robot.Joints(2).error)*Hry*Tx(robot.Links(3).length);
T3Hr =Ry(q(3)+robot.Joints(3).error)*Hry*Tz(robot.Links(4).length)*Tx(robot.Links(5).length);
T4Hr=Rx(q(4)+robot.Joints(4).error)*Hrx;
T5Hr=Ry(q(5)+robot.Joints(5).error)*Hry;
T6Hr=Rx(q(6)+robot.Joints(6).error)*Hrx*Tx(robot.Links(6).length);

Jac1 = T1Hr*T2*T3*T4*T5*T6*tool*[inv(R) [0 0 0]'; 0 0 0 1];
Jac2 = T1*T2Hr*T3*T4*T5*T6*tool*[inv(R) [0 0 0]'; 0 0 0 1];
Jac3 = T1*T2*T3Hr*T4*T5*T6*tool*[inv(R) [0 0 0]'; 0 0 0 1];
Jac4 = T1*T2*T3*T4Hr*T5*T6*tool*[inv(R) [0 0 0]'; 0 0 0 1];
Jac5 = T1*T2*T3*T4*T5Hr*T6*tool*[inv(R) [0 0 0]'; 0 0 0 1];
Jac6 = T1*T2*T3*T4*T5*T6Hr*tool*[inv(R) [0 0 0]'; 0 0 0 1];


  J1 = [Jac1(1:3,4);
      Jac1(3,2);
      Jac1(1,3);
      Jac1(2,1)];

  J2 = [Jac2(1:3,4);
      Jac2(3,2);
      Jac2(1,3);
      Jac2(2,1)];
  J3 = [Jac3(1:3,4);
      Jac3(3,2);
      Jac3(1,3);
      Jac3(2,1)];
  J4 = [Jac4(1:3,4);
      Jac4(3,2);
      Jac4(1,3);
      Jac4(2,1)];
  J5 = [Jac5(1:3,4);
      Jac5(3,2);
      Jac5(1,3);
      Jac5(2,1)];
  J6 = [Jac6(1:3,4);
      Jac6(3,2);
      Jac6(1,3);
      Jac6(2,1)];
  J = [J1, J2, J3, J4, J5, J6];

end