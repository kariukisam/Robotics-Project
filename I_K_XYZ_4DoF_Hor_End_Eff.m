%Inverse Kinematics for thetas given the X, Y, Z following the Equations
%computation. The coordinates are for 3DoF in X_Y FRAME.

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

syms pi;
circle_angle = -pi;

tic
while (circle_angle < pi)

ard = serial('COM6', 'BAUD', 9600); %Initializing the arduino port
fopen(ard);
pause(0.5);

%Calculation of forward kinematics x,y,z coordinates

syms theta alpha a d;
syms theta1 theta2 theta3 theta4;
syms alpha1 alpha2 alpha3 alpha4;
syms a1 a2 a3 a4;
syms d1 d2 d3 d4;
syms pi;
syms x a3;
syms X0 Y0; %For joint5/ End effector with count starting at 0
syms X3 Y3; %For joint 4
syms theta2_1 theta2_2 theta3_1 theta4_1

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

a2 = 10.5;
a3 = 10;
a4 = 7;

d1 = 11; %This becomes the y - intercept
d1x = 0;
intercept = d1;
radius = a4; 

disp(circle_angle);

X = 22;
Y = 0 + 7*sin(circle_angle);
Z = 15 + 7*cos(circle_angle);

%End effector coordinates that you want the end effector to move to
%X = input('Input the coordinate X: ');
%Y = input('Input the coordinate Y: ');
%Z = input('Input the coordinate Z: ');

X0 = X;
Y0 = Y;
Z0 = Z;


%% Section for determining the first joint from the end effector
if (Z0 >= intercept)

XY = sqrt((X0)^2 + (Y0)^2);
disp(XY);

XY_3 = (XY) - a4;
disp(XY_3);

Hyp_XY_3 = sqrt((XY_3)^2 + (Z0 - intercept)^2);
disp(Hyp_XY_3);

theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
disp(theta4_Trap);

theta2_1 = 180 - theta4_Trap;
disp(theta2_1);

theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
theta4_1 = double((theta4_1) * (180/pi));
disp(theta4_1)

theta4 = (theta4_1 - theta2_1);
disp(theta4);

theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
theta3_1 = double((theta3_1) * (180/pi));

if (theta3_1 >= 90)
    theta3 = -(180 - theta3_1);
    theta2_2 = 180 - (theta3_1 + theta4_1);
    theta2 = theta2_1 + theta2_2;
    disp(theta2);
    disp(theta3);
else
    %Down picking
    %theta3 = -90;
    %theta2_2 = 180 - ((90-theta3_1) + theta4_1 + 90);
    %theta2 = theta2_1 + theta2_2;
    %disp(theta2);
    
    XY_3 = (XY);
    disp(XY_3);
    
    Z3 = Z0 + a4;

    Hyp_XY_3 = sqrt((XY_3)^2 + (Z3 - intercept)^2);
    disp(Hyp_XY_3);

    theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
    disp(theta4_Tri);

    theta2_1 = 180 - (theta4_Tri + 90);
    disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    disp(theta4_1)

    theta4 = -(180-(theta4_1 + theta4_Tri));
    disp(theta4);

    if theta4 >= -90
        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        disp(theta3_1);
    
        theta3 = -(180 - theta3_1);
        disp(theta3);
    
        theta2_2 = 180 - (theta4_1 + theta3_1);
        theta2 = theta2_1 + theta2_2;
        disp(theta2);
    else
        
        XY_3 = sqrt((XY)^2 + (Z0 - intercept)^2);
        disp(XY_3);

        Hyp_XY_3 = sqrt((XY_3)^2 + (a4)^2);
        disp(Hyp_XY_3);
        
        theta4 = -90;
        disp(theta4);

        theta2_1 = double((acos((XY)/(XY_3)) * (180/pi)));
        disp(theta2_1);

        theta2_2 = double((acos((XY_3)/(Hyp_XY_3)) * (180/pi)));
        disp(theta2_1);

        theta2_3 = acos(((Hyp_XY_3)^2 + (a2)^2 - (a3)^2) / (2 * Hyp_XY_3 * a2));
        theta2_3 = double((theta2_3) * (180/pi));
        disp(theta2_3)

        theta2 = theta2_1 + theta2_2 + theta2_3;
        disp(theta2);

        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        
        theta3 = -(180 - theta3_1);
        disp(theta3);
        
    end
end

elseif(Z0 <= 4)
    
    XY = sqrt((X0)^2 + (Y0)^2);
    disp(XY);

    XY_3 = (XY) - a4;
    disp(XY_3);

    Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z0)^2);
    disp(Hyp_XY_3);

    theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
    disp(theta4_Trap);

    theta2_1 = 180 - theta4_Trap;
    disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    disp(theta4_1)

    theta4 = (theta4_1 + theta2_1);
    disp(theta4);

    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));

    if (theta3_1 >= 90)
        theta2_2 = 180 - (theta3_1 + theta4_1);
        theta2 = theta2_2 - theta2_1;
        if (isneg(theta2) == 1)
            theta2 = 0;
        end
       
        disp(theta2);
        theta3 = -(180 - theta3_1);
        disp(theta3);
    else
    
        XY_3 = (XY);
        disp(XY_3);
    
        Z3 = Z0 + a4;

        Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z3)^2);
        disp(Hyp_XY_3);

        theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
        disp(theta4_Tri);

        theta2_1 = 180 - (theta4_Tri + 90);
        disp(theta2_1);

        theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
        theta4_1 = double((theta4_1) * (180/pi));
        disp(theta4_1)

        theta4 = -(theta4_Tri - theta4_1);
        disp(theta4);
        
        if theta4 >= -90
            theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
            theta3_1 = double((theta3_1) * (180/pi));
            disp(theta3_1);
    
            theta3 = -(180 - theta3_1);
            disp(theta3);
    
            theta2_2 = 180 - (theta4_1 + theta3_1);
            theta2 = theta2_2 - theta2_1;
            disp(theta2);
        else
            
            XY_3 = sqrt((XY)^2 + (intercept - Z0)^2);
            disp(XY_3);
            
            Hyp_XY_3 = sqrt((XY_3)^2 + (a4)^2);
            disp(Hyp_XY_3);
        
            theta4 = -90;
            disp(theta4);
    
            theta2_1 = double((asin((XY)/(XY_3)) * (180/pi)));
            disp(theta2_1);

            theta2_2 = double((acos((XY_3)/(Hyp_XY_3)) * (180/pi)));
            disp(theta2_1);

            theta2_3 = 90 - (theta2_1 + theta2_2);
            disp(theta2_3);
        
            theta2_4 = acos(((Hyp_XY_3)^2 + (a2)^2 - (a3)^2) / (2 * Hyp_XY_3 * a2));
            theta2_4 = double((theta2_4) * (180/pi));
            disp(theta2_4)

            theta2 = theta2_4 - theta2_3;
            disp(theta2);

            theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
            theta3_1 = double((theta3_1) * (180/pi));
        
            theta3 = -(180 - theta3_1);
            disp(theta3);
        end
    end
    


elseif (Z0 > 4) && (Z0 <= 11)
    XY = sqrt((X0)^2 + (Y0)^2);
    disp(XY);
    
    XY_3 = (XY) - a4;
    disp(XY_3);
    
    Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z0)^2);
    disp(Hyp_XY_3);
    
    theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
    disp(theta4_Trap);
    
    theta2_1 = 180 - theta4_Trap;
    disp(theta2_1);
    
    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    disp(theta4_1)

    theta4 = (theta4_1 + theta2_1);
    disp(theta4);
    
    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));
    
    if (theta3_1 >= 90)
        theta3 = -(180 - theta3_1);
        theta2_2 = 180 - (theta3_1 + theta4_1);
        theta2 = theta2_2 - theta2_1;
        disp(theta2);
        disp(theta3);
    else
        
        XY_3 = (XY);
        disp(XY_3);
        
        Z3 = Z0 + a4;
        
        Hyp_XY_3 = sqrt((XY_3)^2 + (Z3 - intercept)^2);
        disp(Hyp_XY_3);
        
        theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
        disp(theta4_Tri);
        
        theta2_1 = 180 - (theta4_Tri + 90);
        disp(theta2_1);
        
        theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
        theta4_1 = double((theta4_1) * (180/pi));
        disp(theta4_1)
        
        theta4 = -(180 - (theta4_Tri + theta4_1));
        disp(theta4);
    
        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        disp(theta3_1);
    
        theta3 = -(180 - theta3_1);
        disp(theta3);
    
        theta2_2 = 180 - (theta4_1 + theta3_1);
        theta2 = theta2_2 - theta2_1;
        disp(theta2);
    
    end
end



theta1 = double((atan(Y0/X0) * (180/pi)));
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
        position_linear = position_default(:);
        pos_line = position_default(motor_num);
       
    
       %angle_move = input('Angle to move motor: ', 's');
       %angle_move = '0a';
       if (pos_line <= abs(181))
           angle = int2str(pos_line);
           terminator = "a";
           pos_term = angle + terminator;
           angle_move = pos_term;
           pause(0.5);
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
       
       
       else
           disp("Invalid angle. Input value between 0 and 180");
           pause(2);
           fprintf(ard, '90a');
       end
    end
     
    

%%Dislay section and closing the port
    disp("X is: ");
    disp(X);
    disp("Y is: ");
    disp(Y);
    disp("Z is: ");
    disp(Z);
    disp("End");
    

    circle_angle = circle_angle + (pi/4);

     if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
     end

end
toc 


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


