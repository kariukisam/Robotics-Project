%Equation to calculate the inverse kinematics for thetas given the X, Y, Z
%Coordinates in 3-Dimension
%Using the End effector link slope with the Z - intercept

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
syms x a3;
syms X0 Y0 Z0; %For joint5/ End effector with count starting at 0
syms X3 Y3 Z3; %For joint 4
syms theta2_1 theta2_2 theta3_1 theta4_1
syms theta2 theta3 theta4


a2 = 10.5;
a3 = 10;
a4 = 7;

d1 = 11; %This becomes the y - intercept
d1x = 0;
intercept = d1;
radius = a4; 


%End effector coordinates that you want the end effector to move to
X0 = input('Input the coordinate X: ');
Y0 = input('Input the coordinate Y: ');
Z0 = input('Input the coordinate Z: ');


%% Section for determining the first joint from the end effector

%Equation of a circle = Y = MX+C
Hyp_X_Y = sqrt((X0)^2 + (Y0)^2);

slope = ((Z0 - intercept) / (Hyp_X_Y - d1x));
disp(slope);
if(slope ~= inf)
    z = slope*x + intercept;
    disp(z);
elseif(slope == inf)
    x = 0;
    z = intercept;
    slope = 0;
end

%% Angle determination    
if (isneg(slope) == 1)
    Hyp_XY = sqrt((Hyp_X_Y)^2 + (intercept - Z0)^2);
    disp(Hyp_XY);
    
    Hyp_X3Z3 = Hyp_XY - a4;
    disp(Hyp_X3Z3); 
    
    theta4_1 = acos(((Hyp_X3Z3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_X3Z3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    theta4 = theta4_1; %Vertically opposite angles
    disp(theta4);
    
    
    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_X3Z3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));
    %disp(theta3_1);

    theta3 = 180 - theta3_1;
    if(theta3 <= 90)
        theta3 = -theta3;
    else
        disp("Angle too large! Implementing down-up conversion");
        theta3_1 = 90;
        theta3 = 180 - theta3_1;
        theta3 = -theta3;
    end
    
    
    
    disp(Hyp_X3Z3);
    theta2_1 = double((acos(Hyp_X_Y/Hyp_XY)) * (180/pi)); %Converted to degrees
    disp(theta2_1);
    
    theta2_2 = (180 - theta4_1 - theta3_1);
    disp(theta2_2);
    
    theta2 = theta2_2 - theta2_1;
    disp(theta2);
    
else
    Hyp_XY = sqrt((Hyp_X_Y)^2 + (Z0-intercept)^2);
    disp(Hyp_XY);
    
    Hyp_X3Z3 = Hyp_XY - a4;
    disp(Hyp_X3Z3); 
    
    theta4_1 = acos(((Hyp_X3Z3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_X3Z3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    theta4 = theta4_1; %Vertically opposite angles
    disp(theta4);
    
    
    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_X3Z3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));
    %disp(theta3_1);

    theta3 = 180 - theta3_1;
    if(theta3 <= 90)
        theta3 = -theta3;
    else
        disp("Angle too large! Implementing down-up conversion");
        theta3_1 = 90;
        theta3 = 180 - theta3_1;
        theta3 = -theta3;
    end
    
    
    
    theta2_1 = double((acos(Hyp_X_Y/Hyp_XY)) * (180/pi)); %Converted to degrees
    disp(theta2_1);
    
    theta2_2 = (180 - theta4_1 - theta3_1);
    disp(theta2_2);

    theta2 = theta2_1 + theta2_2;
    disp(theta2);
    
end

%Consideration for 3-D: Determining the angle theta1

theta1 = (atan(Y0/X0) * (180/pi));
disp(theta1);
%% Start of Forward_Kinematics after angle calculation

%Single iteration loop to just assign the X,Y,Z coordinates.
    
    [rad1, ~, ~, ~] = deg_to_rad(theta1, theta2, theta3, theta4);
    [~, rad2, ~, ~] = deg_to_rad(theta1, theta2, theta3, theta4);
    [~, ~, rad3, ~] = deg_to_rad(theta1, theta2, theta3, theta4);
    [~, ~, ~, rad4] = deg_to_rad(theta1, theta2, theta3, theta4);
    
    [X, ~, ~] = my_coord(rad1, rad2, rad3, rad4);
    [~, Y, ~] = my_coord(rad1, rad2, rad3, rad4);
    [~, ~, Z] = my_coord(rad1, rad2, rad3, rad4);
    
    X = double(X);
    Y = double(Y);
    Z = double(Z);
    
    X = round(X, 3);
    Y = round(Y, 3);
    Z = round(Z, 3);
    

%% Loop to iterate through the five angles asigned to the position array
    
    for motor_num = 1:5
        %Only changing the angles theta 2 and 3
        angle1 = mapfun(theta1, -90, 90, 0, 180);
        angle2 = mapfun(theta2, 0, 180, 180, 0);
        angle3 = mapfun(theta3, -90, 90, 180, 0);
        angle4 = mapfun(theta4, -90, 90, 180, 0);
        
        position_default = [angle1, angle2, angle3, angle4, 0];
        %position_linear = position_default(:);
        pos_line = position_default(motor_num);
       
    
       %angle_move = input('Angle to move motor: ', 's');
       %angle_move = '0a';
       if (pos_line <= abs(181))
           angle = int2str(pos_line);
           terminator = "a";
           pos_term = angle + terminator;
           angle_move = pos_term;
           pause(2);
           fprintf(ard, angle_move);
       
           format compact;
           pause(0.5);
           readData = fscanf(ard);
           count = fscanf(ard);
       
           format compact;
           disp(readData);
           disp(count);
      
       
       %ret = readData + 0;
       %disp(ret);
           disp('end');
       
       
       else
           disp("Invalid angle. Input value between 0 and 180");
           pause(2);
           fprintf(ard, '90a');
       end
    end
     
    

%% Dislay section and closing the port
    disp("X is: ");
    disp(X);
    disp("Y is: ");
    disp(Y);
    disp("Z is: ");
    disp(Z);
    disp("End");
    

     if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
     end
     
%% My Functions     
%Function to convert the angle assigned to the X,Y coordinates following
%the DH Convention representation relation of the two
function [X, Y, Z] = my_coord(theta1, theta2, theta3, theta4)
a2 = 10.5;
a3 = 10;
a4 = 7;
d1 = 11;
d4 = 0;
alpha1 = (pi/2);

X = a4*cos(theta4)*(cos(theta3)*(cos(theta1)*cos(theta2) - cos(alpha1)*sin(theta1)*sin(theta2)) - sin(theta3)*(cos(theta1)*sin(theta2) + cos(alpha1)*cos(theta2)*sin(theta1))) - a3*sin(theta3)*(cos(theta1)*sin(theta2) + cos(alpha1)*cos(theta2)*sin(theta1)) - a4*sin(theta4)*(cos(theta3)*(cos(theta1)*sin(theta2) + cos(alpha1)*cos(theta2)*sin(theta1)) + sin(theta3)*(cos(theta1)*cos(theta2) - cos(alpha1)*sin(theta1)*sin(theta2))) + a2*cos(theta1)*cos(theta2) + d4*sin(alpha1)*sin(theta1) + a3*cos(theta3)*(cos(theta1)*cos(theta2) - cos(alpha1)*sin(theta1)*sin(theta2)) - a2*cos(alpha1)*sin(theta1)*sin(theta2);
Y = a4*cos(theta4)*(cos(theta3)*(cos(theta2)*sin(theta1) + cos(alpha1)*cos(theta1)*sin(theta2)) - sin(theta3)*(sin(theta1)*sin(theta2) - cos(alpha1)*cos(theta1)*cos(theta2))) - a3*sin(theta3)*(sin(theta1)*sin(theta2) - cos(alpha1)*cos(theta1)*cos(theta2)) - a4*sin(theta4)*(cos(theta3)*(sin(theta1)*sin(theta2) - cos(alpha1)*cos(theta1)*cos(theta2)) + sin(theta3)*(cos(theta2)*sin(theta1) + cos(alpha1)*cos(theta1)*sin(theta2))) - d4*sin(alpha1)*cos(theta1) + a2*cos(theta2)*sin(theta1) + a3*cos(theta3)*(cos(theta2)*sin(theta1) + cos(alpha1)*cos(theta1)*sin(theta2)) + a2*cos(alpha1)*cos(theta1)*sin(theta2);
Z = d1 + d4*cos(alpha1) + a2*sin(alpha1)*sin(theta2) + a4*cos(theta2 + theta3)*sin(alpha1)*sin(theta4) + a4*sin(theta2 + theta3)*sin(alpha1)*cos(theta4) + a3*sin(alpha1)*cos(theta2)*sin(theta3) + a3*sin(alpha1)*cos(theta3)*sin(theta2);
end

%Function to convert degrees to radians for the use in Forward kinematics
%calculation
function [rad1, rad2, rad3, rad4] = deg_to_rad(theta1, theta2, theta3, theta4)
rad1 = double(theta1 / 180 * pi);
rad2 = double(theta2 / 180 * pi);
rad3 = double(theta3 / 180 * pi);
rad4 = double(theta4 / 180 * pi);
end

%Function mapfun created by David Coventry on 1/19/2017 editted and used as
%arduino map function for angle conversion.
function output = mapfun(value, fromLow, fromHigh, toLow,toHigh)
%Function to map the angle input by user to a suitable angle rotation angle
%for use in the arduino movemotor function
narginchk(5,5)
nargoutchk(0,1)
output = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end

function answer = isneg(value)
if (value < 0)
    answer = 1;
else
    answer = 0;
end
end
