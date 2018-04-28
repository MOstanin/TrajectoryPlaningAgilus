% Advanced Robotics Manipulation
% KUKA Agilus
% Mikhail Ostanin, Innopolis 2018

%% Robot model description
%%% Link description
% link.type      - specify this part as a link with 'L' value
% link.length    - length of the link in meters
% link.axis      - orientation of the link in initial pose [X Y Z], so [1 0 0]
%                  corresponds to link with 10m length in X direction
% link.mass      - mass of the link (for dynamic calculation in future)
% link.inertia   - inertia matrix
% link.stiffness - stiffness matrix
% link.twist     - additional twist of the link
% link.diamert   - diamert size of link
% link.thickness - difference between external and internal radiuses

link1.type = 'L';
link1.length = 0.4; 
link1.axis = [0 0 1];

link2.type = 'L';
link2.length = 25*10^(-3);
link2.axis = [1 0 0];

link3.type = 'L';
link3.length = 0.56;
link3.axis = [1 0 0];

link4.type = 'L';
link4.length = 35*10^(-3);
link4.axis = [0 0 1];

link5.type = 'L';
link5.length = 0.515;
link5.axis = [1 0 0];

link6.type = 'L';
link6.length = 0.08;
link6.axis = [1 0 0];

%%% Joint description
% joint.type     - specify this part as a joint with 'P' or 'R' value
%                  (revolute or prismatic joint)
% joint.axis     - axis of rotation in base frame [X Y Z], so [1 0 0]
%                  corresponds to joint with rotation about X axis
% joint.limit    - lower and upper limit for joint angle (distance)
% joint.child    - array with child links
% joint.parent   - array with parent links
% joint.position - coordinates of the joint
% joint.stiffness - stiffness value

joint1.type = 'R';
joint1.axis = [0 0 1];
joint1.limit = [-pi pi];
joint1.parent = ['base'];
joint1.child = ['link1'];
joint1.position = [0;0;0];
joint1.error = 0;

joint2.type = 'R';
joint2.axis = [0 1 0];
joint2.limit = [-pi pi];
joint2.parent = ['link2'];
joint2.child = ['link3'];
joint2.position = [0;0;0];     % not used
joint2.error = 0;

joint3.type = 'R';
joint3.axis = [0 1 0];
joint3.limit = [-pi pi];
joint3.parent = ['link3'];
joint3.child = ['link4'];
joint3.position = [0;0;0];     % not used
joint3.error = 0;

joint4.type = 'R';
joint4.axis = [1 0 0];
joint4.limit = [-pi pi];
joint4.parent = ['link5'];
joint4.child = ['link6'];
joint4.position = [0;0;0];
joint4.error = 0;

joint5.type = 'R';
joint5.axis = [0 1 0];
joint5.limit = [-pi pi];
joint5.parent = ['link5'];
joint5.child = ['link6'];
joint5.position = [0;0;0];    % not used
joint5.error = 0;

joint6.type = 'R';
joint6.axis = [1 0 0];
joint6.limit = [-pi pi];
joint6.parent = ['link5'];
joint6.child = ['link6'];
joint6.position = [0;0;0];    % not used
joint6.error = 0;

%%% Robot assembly
% robot.Name - take a guess
% robot.Links - array with links
% robot.Joints - array with joints

r3_robot.Name = 'KUKA Agilus';
r3_robot.Links = [link1 link2 link3 link4 link5 link6];
r3_robot.Joints = [joint1 joint2 joint3 joint4 joint5 joint6];

clear link1 link2 link3 link4 link5 link6
clear joint1 joint2 joint3 joint4 joint5 joint6

