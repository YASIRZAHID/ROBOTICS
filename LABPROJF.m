clear all
clc
%Forward Kinematics
a=[0 0 220 220 0 0] % link lengths
al=[0 pi/2 0 0 pi/2 0] % link twists
d=[385 0 0 0 0 155] % link offsets
s=[-pi/2 0 pi/2 0 0 0] % joint angles
 
c1=[cos(s(1)) cos(s(2)) cos(s(3)) cos(s(4)) cos(s(5)) cos(s(6))] %taking cos of joint angles
s1=[sin(s(1)) sin(s(2)) sin(s(3)) sin(s(4)) sin(s(5)) sin(s(6))] %taking sin of joint angles
c2=[cos(al(1)) cos(al(2)) cos(al(3)) cos(al(4)) cos(al(5)) cos(al(6))] %taking cos of link twist angles
s2=[sin(al(1)) sin(al(2)) sin(al(3)) sin(al(4)) sin(al(5)) sin(al(6))] %taking sin of link twist angles
%Transform Matrixes (i-1)Ti
T1=[c1(1) -s1(1)*c2(1) s1(1)*s2(1) a(1)*c1(1); s1(1) c1(1) -c1(1)*s2(1) a(1)*s2(1); 0 s2(1) c2(1) d(1); 0 0 0 1]%0T1
T2=[c1(2) -s1(2)*c2(2) s1(2)*s2(2) a(2)*c1(2); s1(2) c1(2) -c1(2)*s2(2) a(2)*s2(2); 0 s2(2) c2(2) d(2); 0 0 0 1]%1T2
T3=[c1(3) -s1(3)*c2(3) s1(3)*s2(3) a(3)*c1(3); s1(3) c1(3) -c1(3)*s2(3) a(3)*s2(3); 0 s2(3) c2(3) d(3); 0 0 0 1]%2T3
T4=[c1(1) -s1(4)*c2(4) s1(4)*s2(4) a(4)*c1(4); s1(4) c1(4) -c1(4)*s2(4) a(4)*s2(4); 0 s2(4) c2(4) d(4); 0 0 0 1]%3T4
T5=[c1(2) -s1(5)*c2(5) s1(5)*s2(5) a(5)*c1(5); s1(5) c1(5) -c1(5)*s2(5) a(5)*s2(5); 0 s2(5) c2(5) d(5); 0 0 0 1]%4T5
T6=[c1(3) -s1(6)*c2(6) s1(6)*s2(6) a(6)*c1(6); s1(6) c1(6) -c1(6)*s2(6) a(6)*s2(6); 0 s2(6) c2(6) d(6); 0 0 0 1]%5T6
T=T1*T2*T3*T4*T5*T6; %Transform matrix after multiplying all transformation from 0 joint to 4 (0T4)
%declaring Links using link function from libray
L(1) = Link([s(1),d(1),a(1),al(1)])
L(2) = Link([s(2),d(2),a(2),al(2)])
L(3) = Link([s(3),d(3),a(3),al(3)])
L(4) = Link([s(4),d(4),a(4),al(4)])
L(5) = Link([s(5),d(5),a(5),al(5)])
L(6) = Link([s(6),d(6),a(6),al(6)])
SCORBOT = SerialLink(L) %joining links together to form manipulator using SerialLink command
T = fkine(SCORBOT, [s(1) s(2) s(3) s(4) s(5) s(6)]) %computing forward kinematics
SCORBOT.plot([s(1) s(2) s(3) s(4) s(5) s(6)]) %ploting Scorbot manipulator which we form
title ("ROBOTIC ARM ")