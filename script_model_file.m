%REMEMBER: if you need to change the lenght of the cable
%or the block dimensions go in mdl_pendl() %to use this function you have to instal the Robotic Toolbox by Peter Coorke


clc,close all,clear all


syms q1 q2 q3 q4 q5 real %position 
syms qd1 qd2 qd3 qd4 qd5 real %velocity


mdl = mdl_pendl(); %model of the pendulum
sym(mdl) %the model is converted in a symbolic form 

q = [q1 q2 q3 q4 q5]'; %position vector
qd = [qd1 qd2 qd3 qd4 qd5]'; %velocity vector
 
x = [q;qd]; %state vector
B = mdl.inertia(q'); %Inertia Matrix in symbolic 
C = mdl.coriolis(q',qd'); %Coriolis Matrix in symbolic
G = mdl.gravload(q'); % Gravity vector in symbolic


B_inv = inv(B);

q2d = B_inv*(-C*qd -G'); %equation of the motion
dx = [qd; q2d];


%code to lineaize the model

x_bar = zeros(10,1);
u_bar = zeros(5,1);


A_ss = double(subs(jacobian(dx, x), x, x_bar));
B_ss = zeros(10,1);
C_ss = zeros(5,10); C_ss(1:5,1:5) = eye(5);
D_ss = zeros(5,1);


save('lin_mdl','A_ss','B_ss','C_ss','D_ss');

[Ad,Bd,Cd,Dd]=ssdata(c2d(ss(A_ss,B_ss,C_ss,D_ss),0.0033,'ZOH'));

A_ss = Ad;

fileID = fopen('Ad.txt','w');
formatSpec = '%8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f\n';
fprintf(fileID,formatSpec,A_ss');

