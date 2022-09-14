function [r_e,p_e] = dir_kin(u)

q = u(1:5,1);

l = 1.215;  %length cable
b = 0.3;  %heigth_block/2

q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5);

%%%%%%%%%%%%% T matrix from the fix frame to the spherical joint
T_02 = [ cos(q2)*sin(q1), -sin(q1)*sin(q2),  cos(q1), -l*cos(q2)*sin(q1);
        -sin(q2),         -cos(q2),        0,          l*sin(q2);
 cos(q1)*cos(q2), -cos(q1)*sin(q2), -sin(q1), -l*cos(q1)*cos(q2);
               0,                0,        0,                  1];
%%%% T matrix from the spherical joint to the blok COG

T_25 = [ - sin(q3)*sin(q5) - cos(q3)*cos(q5)*sin(q4), cos(q3)*sin(q4)*sin(q5) - cos(q5)*sin(q3), -cos(q3)*cos(q4), -(b*10*cos(q3)*cos(q4))/10;
   cos(q3)*sin(q5) - cos(q5)*sin(q3)*sin(q4), cos(q3)*cos(q5) + sin(q3)*sin(q4)*sin(q5), -cos(q4)*sin(q3), -(b*10*cos(q4)*sin(q3))/10;
                             cos(q4)*cos(q5),                          -cos(q4)*sin(q5),         -sin(q4),         -(b*10*sin(q4))/10;
                                           0,                                         0,                0,                       1];
 
%%%%%%

T_05 = T_02*T_25;

[r_e,p_e] = tr2rt(T_05); %get from the T matrix the Rot Matrix (r_e) and the position (p_e) 




end