function g=jaco2_g(q)
%
% Computes the gravity torques for the KINOVA Jaco2
%
% function g = jaco2_g(g)
%
% check the file jaco2Info_ver2.pdf for the definitions
%
% input:
%       q     dim 7x1     joint positions in DH convention
%
% output:
%       g     dim 7x1     gravity torques
%
% Gianluca Antonelli - Introduction to robotics, 2022/2023

    s2 = sin(q(2));
    s3 = sin(q(3));
    s4 = sin(q(4));
    s5 = sin(q(5));
    s6 = sin(q(6));

    c2 = cos(q(2));
    c3 = cos(q(3));
    c4 = cos(q(4));
    c5 = cos(q(5));
    c6 = cos(q(6));

    g1=0;
    g2=(817529685764214447*c4*s2)/225179981368524800 - (572555641444887573*c2*s3)/3602879701896396800 - (8448257495169353187*s2)/900719925474099200 - (817529685764214447*c2*c3*s4)/225179981368524800 - (2991791788070197707*c4*c6*s2)/7205759403792793600 + (2991791788070197707*c2*c3*c6*s4)/7205759403792793600 - (2991791788070197707*c2*s3*s5*s6)/7205759403792793600 - (2991791788070197707*c5*s2*s4*s6)/7205759403792793600 - (2991791788070197707*c2*c3*c4*c5*s6)/7205759403792793600;
    g3=-(981*s2*(1167289788878466*c3 - 26667635009637984*s3*s4 + 3049736787023647*c6*s3*s4 + 3049736787023647*c3*s5*s6 - 3049736787023647*c4*c5*s3*s6))/7205759403792793600;
    g4=(817529685764214447*c2*s4)/225179981368524800 - (817529685764214447*c3*c4*s2)/225179981368524800 - (2991791788070197707*c2*c6*s4)/7205759403792793600 + (2991791788070197707*c3*c4*c6*s2)/7205759403792793600 + (2991791788070197707*c2*c4*c5*s6)/7205759403792793600 + (2991791788070197707*c3*c5*s2*s4*s6)/7205759403792793600;
    g5=-(3049736787023647*s6*(s5*((981*c2*s4)/100 - (981*c3*c4*s2)/100) + (981*c5*s2*s3)/100))/72057594037927936;
    g6=(3049736787023647*c6*(c5*((981*c2*s4)/100 - (981*c3*c4*s2)/100) - (981*s2*s3*s5)/100))/72057594037927936 - (3049736787023647*s6*((981*c2*c4)/100 + (981*c3*s2*s4)/100))/72057594037927936;
    g7=0;

    g=[g1; g2; g3; g4; g5; g6; g7];

end