%Calculation of forward kinematics x,y,z coordinates
%This is given the motor angle positions.
%By the DH Convention, the coordinates can be determined from the 
%link length, offset, twist and the angles.

syms theta alpha a d;
syms theta1 theta2 theta3 theta4;
syms alpha1 alpha2 alpha3 alpha4;
syms a1 a2 a3 a4;
syms d1 d2 d3 d4;
syms pi;

c0 = cos(theta);
ca = cos(alpha);
c1 = cos(theta1);
c2 = cos(theta2);
c3 = cos(theta3);
c4 = cos(theta4);
ca1 = cos(alpha1);

s0 = sin(theta);
sa = sin(alpha);
s1 = sin(theta1);
s2 = sin(theta2);
s3 = sin(theta3);
s4 = sin(theta4);
sa1 = sin(alpha1);

T = [c0 -s0*ca s0*sa a*c0;
    s0 c0*ca -c0*sa a*s0;
    0 sa ca d;
    0 0 0 1];
f(theta, alpha, d, a) = T;


T1 = zeros(4);
T2 = zeros(4);
T3 = zeros(4);
T4 = zeros(4);

angles = 1;

while (angles < 5)
    
    %position_default = [100, 90, 120, 180];
    %position_linear = position_default(:);
    %pos_line = position_default(angles);
    
    %value = zeros(1);
    %value(angles) = pos_line;
    
    if (angles == 1)
        %theta1 = ((value(1)/180) * pi);
        theta1 = theta1;
        alpha1 = alpha1;
        d1 = d1;
        a1 = a1;
        
        T1 = f(theta1, alpha1, d1, a1);
        
    elseif (angles == 2)
        %theta2 = ((value(2)/180) * pi);
        theta2 = theta2;
        alpha = 0;
        d2 = 0;
        a2 = a2;
        
        T2 = f(theta2, alpha, d2, a2);
    elseif (angles == 3)
        %theta3 = ((value(3)/180) * pi);
        theta3 = theta3;
        alpha = 0;
        d3 = 0;
        a3 = a3;
        
        T3 = f(theta3, alpha, d3, a3);
    elseif (angles == 4)
        %theta4 = ((value(4)/180) * pi);
        theta4 = theta4;
        alpha = 0;
        d4 = d4;
        a4 = a4;
        
        T4 = f(theta4, alpha, d4, a4);
    end        
    
    angles = angles + 1;
    
end

T04 = T1 * T2 * T3 * T4;
T04 = simplify(T04);
T04_linear = T04(:);
X = T04_linear(13);
Y = T04_linear(14);
Z = T04_linear(15);

format compact;
disp(T04);

disp(X);
disp(Y);
disp(Z);
