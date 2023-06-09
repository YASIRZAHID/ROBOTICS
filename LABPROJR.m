%% Trajectories in the joint space
clear clc
close all
%define the arms of the cylindrical MP according
%to the parameters of DH
%Link[s d a a1 0]
L(1)=Link([-pi/2 385 0 0 0],'standard');
L(2)=Link([0 0 0 pi/2 0],'standard');
L(3)=Link([pi/2 0 220 0 0],'standard');
L(4)=Link([0 0 220 0 0],'standard');
L(5)=Link([0 0 0 pi/2 0],'standard');
L(6)=Link([0 155 0 0 0],'standard');
q_lim=[-31*pi/36 31*pi/36 ;-pi/4 pi/4;-pi/2 0.4555*pi;-13*pi/18 13*pi/18;-pi pi;0 0];
q=[0 0 0 0 0 0];
%let's define our robot with the SerialLink command
MPc=SerialLink(L,'name','MPc','qlim',q_lim);
%plot the cylindrical MP in the workspace
% MPc.plot(q,'tilesize',10);
theta1=linspace(-31*pi/36,31*pi/36,8);
theta2=linspace(-pi/4,pi/4,8);
theta3=linspace(-pi/2,0.4555*pi,8);
theta4=linspace(-13*pi/18,13*pi/18,8);
theta5=linspace(-pi,pi,8);
theta6=linspace(0,0,8);
q0=[theta1(1) theta2(1) theta3(1) theta4(1) theta5(1) theta6(1)];
q1=[theta1(2) theta2(2) theta3(2) theta4(2) theta5(2) theta6(2)];
q2=[theta1(3) theta2(3) theta3(3) theta4(3) theta5(3) theta6(3)];
q3=[theta1(4) theta2(4) theta3(4) theta4(4) theta5(4) theta6(4)];
q4=[theta1(5) theta2(5) theta3(5) theta4(5) theta5(5) theta6(5)];
q5=[theta1(6) theta2(6) theta3(6) theta4(6) theta5(6) theta6(6)];
q6=[theta1(7) theta2(7) theta3(7) theta4(7) theta5(7) theta6(7)];
% consider a parameter for each trajectory
% Trajectory - 0
jtraj0 = jtraj(q0,q1,20)
% generates the trajectory in the joint space
% In addition, the speed values à and acceleration are
% calculated with a seventh degree polynomial
% with null boundary conditions [q dq ddq ]
q_f0 = MPc.fkine(jtraj0)
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f0(1,i).t(1);
y_traj(1,i) = q_f0(1,i).t(2);
z_traj(1,i) = q_f0(1,i).t(3);
end
%assign the first element to each column of x_traj
% Of the last colomn n to the direct kinematic function
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% the trajectory is indicated by points
% consider a parameter for each trajectory
% Trajectory - I
jtraj1 = jtraj(q1,q2,20)
% generates the trajectory in the joint space
% In addition, the speed values à and acceleration are
% calculated with a seventh degree polynomial
% with null boundary conditions [q dq ddq ]
q_f1 = MPc.fkine(jtraj1)
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f1(1,i).t(1);
y_traj(1,i) = q_f1(1,i).t(2);
z_traj(1,i) = q_f1(1,i).t(3);
end
%assign the first element to each column of x_traj
% Of the last colomn n to the direct kinematic function
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% the trajectory is indicated by points
% Trajectory - II
jtraj2 = jtraj(q2,q3,20);
q_f2 = MPc.fkine(jtraj2);
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f2(1,i).t(1);
y_traj(1,i) = q_f2(1,i).t(2);
z_traj(1,i) = q_f2(1,i).t(3);
end
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% Trajectory -III
jtraj3 = jtraj(q3,q4,20);
q_f3 = MPc.fkine(jtraj3);
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f3(1,i).t(1);
y_traj(1,i) = q_f3(1,i).t(2);
z_traj(1,i) = q_f3(1,i).t(3);
end
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% Trajectory -IV
jtraj4 = jtraj(q4,q5,20);
q_f4 = MPc.fkine(jtraj4);
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f4(1,i).t(1);
y_traj(1,i) = q_f4(1,i).t(2);
z_traj(1,i) = q_f4(1,i).t(3);
end
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% Trajectory -IV
jtraj5 = jtraj(q5,q6,20);
q_f5 = MPc.fkine(jtraj5);
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f5(1,i).t(1);
y_traj(1,i) = q_f5(1,i).t(2);
z_traj(1,i) = q_f5(1,i).t(3);
end
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% Trajectory -Vi
jtraj6 = jtraj(q6,q0,20);
q_f6 = MPc.fkine(jtraj6);
x_traj = zeros(1,20);
y_traj = zeros(1,20);
z_traj = zeros(1,20);
for i=1:20
x_traj(1,i) = q_f6(1,i).t(1);
y_traj(1,i) = q_f6(1,i).t(2);
z_traj(1,i) = q_f6(1,i).t(3);
end
hold on
scatter3(x_traj,y_traj,z_traj,'.');
% Simulation of Results
vstup = 0;
while (vstup ~= 1)
 
MPc.plot(jtraj0)
MPc.plot(jtraj1)
MPc.plot(jtraj2)
MPc.plot(jtraj3)
MPc.plot(jtraj4)
MPc.plot(jtraj5)
MPc.plot(jtraj6)
end
