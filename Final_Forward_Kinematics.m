%This is the code that maps the angle input as the theta 
%such that it is converted to suit the DH representation of angles
%The mapping is done to convert the angles to a suitable representation.


if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

ard = serial('COM6', 'BAUD', 9600); %Initializing the arduino port
fopen(ard);

%Calculation of forward kinematics x,y,z coordinates

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

d1 = 11;
theta1 = 90;
theta2 = 45;
theta3 = 90;
theta4 = 90;
theta5 = 0;

angles = 1;

while (angles < 3)
    
    [rad1, ~, ~, ~] = deg_to_rad(theta1, theta2, theta3, theta4);
    [~, rad2, ~, ~] = deg_to_rad(theta1, theta2, theta3, theta4);
    [~, ~,rad3, ~] = deg_to_rad(theta1, theta2, theta3, theta4);
    [~, ~, ~, rad4] = deg_to_rad(theta1, theta2, theta3, theta4);
    
    [X, ~] = my_coord(rad2, rad3);
    [~, Y] = my_coord(rad2, rad3);
    
    X = double(X);
    Y = double(Y) + d1;
    
    angles = angles + 1;
end

times = 0;

while (times <5)
    
    for motor_num = 1:5
        %Only changing the angles theta 2 and 3
        angle2 = mapfun(theta2, 0, 180, 180, 0);
        angle3 = mapfun(theta3, -90, 90, 180, 0);
        
        position_default = [90, angle2, angle3, 90, 0];
        position_linear = position_default(:);
        pos_line = position_default(motor_num);
       
    
       %angle_move = input('Angle to move motor: ', 's');
       %angle_move = '0a';
       if (pos_line <= abs(180))
       angle = int2str(pos_line);
       terminator = "a";
       pos_term = angle + terminator;
       angle_move = pos_term;
       pause(2);
       fprintf(ard, angle_move);
       
       format compact;
       readData = fscanf(ard);
       count = fscanf(ard);
       
       format compact;
       disp(readData);
       disp(count);
      
       
       %ret = readData + 0;
       %disp(ret);
       disp('end');
       
       times = times +1;
       
       else
           disp("Invalid angle. Input value between 0 and 180");
           pause(2);
           fprintf(ard, '90a');
       end
    end
     
    
     if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
     end
end 

    disp("X is: ");
    disp(X);
    disp("Y is: ");
    disp(Y);
    disp("End");

     if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
     end

function [X, Y] = my_coord(theta2, theta3)
a2 = 10.5;
a3 = 17;

X = a3*cos(theta2 + theta3) + a2*cos(theta2);
Y = a3*sin(theta2 + theta3) + a2*sin(theta2);
end

function [rad1, rad2, rad3, rad4] = deg_to_rad(theta1, theta2, theta3, theta4)
rad1 = double(theta1 / 180 * pi);
rad2 = double(theta2 / 180 * pi);
rad3 = double(theta3 / 180 * pi);
rad4 = double(theta4 / 180 * pi);
end

%Function mapfun created by David Coventry on 1/19/2017 editted and used as
%arduino map function for angle conversion.
function output = mapfun(value, fromLow, fromHigh, toLow,toHigh)
narginchk(5,5)
nargoutchk(0,1)
output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end

