%Inverse Kinematics for thetas given the X, Y, Z following the Equations
%computation. The coordinates are for 3DoF in X_Y FRAME.

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

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

%% Image Processing

matrix_X = zeros(15, 2);
matrix_Y = zeros(15, 2);
% matrix_XY_2 = zeros(15, 4);
% matrix_XY = zeros(30, 2);

sort_XY = zeros(30,2);

%Read image
% Triangle = imread('Annotation 2020-02-24 010557.png'); %Triangle
% Triangle = imread('Annotation 2020-02-26 184222.png'); %Square
% Triangle = imread('Annotation 2020-02-26 184538.png'); %Rectangle
% Triangle = imread('Annotation 2020-02-26 184740.png'); %Up pointer
Triangle = imread('Annotation 2020-02-26 184900.png'); %Plus
% Triangle = imread('Annotation 2020-03-09 172220.jpg'); %Plus scaled


% Triangle = imread('Triangle.jpg');
% Triangle = imread('IMG_20200224_000227.jpg');

%Triangle = imrotate(Triangle, -90);
% subplot(2,3,1);
figure, imshow(Triangle),
% title('Input Image');

%Grayscale conversion
gra=rgb2gray(Triangle);
% subplot(2,3,2);
figure, imshow(gra),
% title('Gray Image');

%Sharpening
gra = imsharpen(gra);

%Binarizing
%threshold = graythresh(gra);
threshold = 0.5;
BW = imbinarize(gra, threshold);
% subplot(2,3,3);
figure, imshow(BW),
% title('Binary Image');

%Complementing
BW_inv = ~BW;
% subplot(2,3,5);
figure, imshow(BW_inv),
title('Inverted Binary Image');

%Filling the holes
Tri_fill = imfill(BW_inv, 'holes');
% subplot(2,3,4);
figure, imshow(Tri_fill),
% title('Filled Image');

% %masking to get back the grayscale image
% Tri_mask = uint8(Tri_fill) .* gra;
% % subplot(2,3,6);
% imshow(Tri_mask),
% title('Masked Image');

%% Edge detection section

[Tri_canny, threshout] = edge(Tri_fill, 'canny', []);
figure, imshow(Tri_canny);
disp(threshout);

%For hough transform determination
[H,T,R] = hough(Tri_canny);
% figure, imshow(H,[], 'XData', T, 'YData',R,...
%     'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;

P = houghpeaks(H,10,'threshold', ceil(0.3*max(H(:))));
x = T(P(:,2));
y = R(P(:,1));
plot(x,y,'s', 'color', 'white');

lines = houghlines(Tri_canny,T,R,P,'FillGap',10,'MinLength',5);
figure, imshow(gra), hold on
max_len = 0;
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    plot(xy(:,1),xy(:,2), 'LineWidth', 2, 'Color', 'blue');
    
    plot(xy(1,1),xy(1,2), 'x', 'LineWidth', 2, 'Color', 'green');
    plot(xy(2,1),xy(2,2), 'x', 'LineWidth', 2, 'Color', 'red');
    
    len = norm(lines(k).point1 - lines(k).point2);
    if(len > max_len)
        max_len = len;
        xy_long = xy;
    end
    
    num = length(lines);
    
    matrix_X(k,1) = xy(1,1);
    matrix_X(k,2) = xy(2,1);
    
    matrix_Y(k,1) = xy(1,2);
    matrix_Y(k,2) = xy(2,2);
end


%% Section for shape reconstruction

%Defining the start and end XYs
matrix_XY_2 = [matrix_X(:,1), matrix_Y(:,1), matrix_X(:,2),matrix_Y(:,2)];

%The matrix X and Y in a single column for start and end
matrix_x_2 = [matrix_X(:,1); matrix_X(:,2)];
matrix_y_2 = [matrix_Y(:,1); matrix_Y(:,2)];

%Getting a two column XY matrix for all the points
matrix_XY = [matrix_x_2 , matrix_y_2];
matrix_XY( ~any(matrix_XY,2),:) = [];


%Sorting the points in decreasing order of Y
current_row = 1;
matrix_XY2 = matrix_XY;

for iterations = 1:30
    Ind = find(matrix_XY2(:,2) == max(matrix_XY2(:,2)));
    indices = length(Ind);
    for ind = 1 : indices
        sort_XY(current_row,2) = matrix_XY2(Ind(ind,1),2);
        sort_XY(current_row,1) = matrix_XY2(Ind(ind,1),1);
    
        current_row = current_row + 1;
    end

    for ind = indices:-1:1 
        %Implement decrementation to avoid erasing the (index +1) after first loop deleting first row
        matrix_XY2(Ind(ind,1),:) = [];
    end
end

%Separating the points into two in reference to the centre X point to have
%ordered coordinates.
sort_XY( ~any(sort_XY,2),:) = [];

max_1 = max(sort_XY(:,1));
min_1 = min(sort_XY(:,1));
mid_1 = (min_1 + max_1)/2;

curr_row_1 = 1;
curr_row_2 = 1;

sort_XY_2 = sort_XY;
XY_mat_1 = zeros(30,2);
XY_mat_2 = zeros(30,2);

%Iterating through the sorted matrix to get the two holder matrices
for number = 1:30
    Ind_2 = find(sort_XY_2(:,2) == max(sort_XY_2(:,2)));
    indices_2 = length(Ind_2);
    
    for ind_2 = 1 : indices_2
        if (sort_XY_2(Ind_2((ind_2),1),1) > mid_1)
            XY_mat_1(curr_row_1, 1) = sort_XY_2(Ind_2((ind_2),1),1);
            XY_mat_1(curr_row_1, 2) = sort_XY_2(Ind_2((ind_2),1),2);
            curr_row_1 = curr_row_1 + 1;
        
        else
            XY_mat_2(curr_row_2, 1) = sort_XY_2(Ind_2((ind_2),1),1);
            XY_mat_2(curr_row_2, 2) = sort_XY_2(Ind_2((ind_2),1),2);
            curr_row_2 = curr_row_2 + 1;
        end
        
    end
    for ind_2 = indices_2:-1:1
        sort_XY_2(Ind_2(ind_2,1),:) = [];
    end      
        
end

%%

%Execution of the continuos points for the shape reconstruction by finding
%for the next nearly identical point.
%Helps in filtering out the midline points.

final_XY = zeros(35,2);
start = [matrix_XY_2(1,1), matrix_XY_2(1,2)];
next = [matrix_XY_2(1,3), matrix_XY_2(1,4)];

rep = 3;
final_XY(rep-2,:) = [matrix_XY_2(1,1), matrix_XY_2(1,2)];
final_XY(rep-1,:) = [matrix_XY_2(1,3), matrix_XY_2(1,4)];

matrix_XY_2( ~any(matrix_XY_2,2),:) = [];

temp = matrix_XY_2;
temp(1,:) = [];

thresh = 13;
for looping = 1:(size(matrix_XY_2,1)-1)
    
    for j  = 1 : (size(temp,1))
%         %disp(j);
        if ((((temp(j,1)) <= (next(1,1)+thresh)) && ((temp(j,1))>= (next(1,1)-thresh)))...
                && (((temp(j,2)) <= (next(1,2)+thresh)) && ((temp(j,2))>=(next(1,2)-thresh))))
            
            final_XY(rep,:) = temp(j,1:2);
            final_XY((rep+1),:) = temp(j,3:4);
            next = temp(j,3:4);
            rep = rep + 2;
            del_row = j;
            break
                
        elseif((((temp(j,3)) <= (next(1,1)+thresh)) && ((temp(j,3))>= (next(1,1)-thresh)))...
                && (((temp(j,4)) <= (next(1,2)+thresh)) && ((temp(j,4))>=(next(1,2)-thresh))))
            
            final_XY(rep,:) = temp(j,3:4);
            final_XY((rep+1),:) = temp(j,1:2);
            next = temp(j,1:2);
            rep = rep + 2;
            del_row = j;
            
            break
        end
             
    end
    
    temp(del_row,:) = [];
end

final_XY( ~any(final_XY,2),:) = [];

%%
%Preparing the handing over to the manipulator
[Y,X,Z] = size(Triangle);

Y_coord = zeros(1, length(final_XY));
Z_coord = zeros(1, length(final_XY));


for i = 1: length(final_XY)
    Y_coord(1,i) =  mapfun(final_XY(i,1), 1, X, -10, 10);
    Z_coord(1,i) = mapfun(final_XY(i,2), 1, Y, 29, 11);
end

% % Enlarging the transformed points
% % about the centre point by using a relative scaling factor
% 
% max_y = max(abs(Y_coord));
% min_y = min(abs(Y_coord));
% 
% min_z = min(abs(Z_coord));
% max_z = max(abs(Z_coord));
% 
% Y_coord_2 = zeros(1, length(final_XY));
% Z_coord_2 = zeros(1, length(final_XY));
% 
% for i = 1: length(final_XY)
%     if max_y >= min_y
%         Y_coord_2(1,i) = mapfun(Y_coord(1,i), -max_y, max_y, -10, 10);
%     else
%         Y_coord_2(1,i) = mapfun(Y_coord(1,i), -min_y, min_y, -10, 10);
%     end
%     
%     if ((31 - max_z) <= (min_z - 11))
%         Z_coord_2(1,i) = mapfun(Z_coord(1,i), (11 + (31-max_z)), max_z , 11, 31);
%     else
%         Z_coord_2(1,i) = mapfun(Z_coord(1,i), min_z, (31 - (min_z - 11)), 11, 31);
%     end
% end

    
    
%% Manipulator handing over

X_in = 22;
Y_in = Y_coord;
Z_in = Z_coord;


% X_in = -22;
% Y_in = [-10  10   10  -10];
% Z_in = [ 11   11  31  11];

X0 = X_in;

points = size(Y_in,2);
%disp(points);

tic
for i = 2:points

%Calculation of forward kinematics x,y,z coordinates

Y1 = Y_in(i - 1);
Y2 = Y_in(i);

Z1 = Z_in(i - 1);
Z2 = Z_in(i);

steps_num = 1;
level = 1;

for times = 1 : level : steps_num
    
ard = serial('COM6', 'BAUD', 9600); %Initializing the arduino port
fopen(ard);
pause(0.5);

step_y = (Y2 - Y1)/steps_num;
step_z = (Z2 - Z1)/ steps_num;
grad = (Z2 - Z1)/(Y2 - Y1);
%disp(grad);
%disp(step_y)

if (Y2 == Y1)
    Z0 = (Z1 + step_z * times);
    Y0 = Y1;
    %disp(X0);
%disp(Y0);
%disp(Z0);
    
elseif(Z1 == Z2)
    Y0 = (Y1 + step_y * times);
    Z0 = Z1; 
    %disp(X0);
%disp(Y0);
%disp(Z0);
    
else
    Y0 = (Y1 + step_y * times);
    c = (Z1 - (grad * Y1));
    Z0 = ((grad * Y0) + c);
    %disp(X0);
%disp(Y0);
%disp(Z0);
end

%% Section for determining the first joint from the end effector

if(isneg(X0) == 0)
    
if (Z0 >= intercept)

XY = sqrt((X0)^2 + (Y0)^2);
%disp(XY);

XY_3 = (XY) - a4;
%disp(XY_3);

Hyp_XY_3 = sqrt((XY_3)^2 + (Z0 - intercept)^2);
%disp(Hyp_XY_3);

theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
%disp(theta4_Trap);

theta2_1 = 180 - theta4_Trap;
%disp(theta2_1);

theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
theta4_1 = double((theta4_1) * (180/pi));
%disp(theta4_1)

theta4 = (theta4_1 - theta2_1);
%disp(theta4);

theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
theta3_1 = double((theta3_1) * (180/pi));

if (theta3_1 >= 90)
    theta3 = -(180 - theta3_1);
    theta2_2 = 180 - (theta3_1 + theta4_1);
    theta2 = theta2_1 + theta2_2;
    %disp(theta2);
    %disp(theta3);
else
    %Down picking
    %theta3 = -90;
    %theta2_2 = 180 - ((90-theta3_1) + theta4_1 + 90);
    %theta2 = theta2_1 + theta2_2;
    %%disp(theta2);
    
    XY_3 = (XY);
    %disp(XY_3);
    
    Z3 = Z0 + a4;

    Hyp_XY_3 = sqrt((XY_3)^2 + (Z3 - intercept)^2);
    %disp(Hyp_XY_3);

    theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
    %disp(theta4_Tri);

    theta2_1 = 180 - (theta4_Tri + 90);
    %disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    %disp(theta4_1)

    theta4 = -(180-(theta4_1 + theta4_Tri));
    %disp(theta4);

    if theta4 >= -90
        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        %disp(theta3_1);
    
        theta3 = -(180 - theta3_1);
        %disp(theta3);
    
        theta2_2 = 180 - (theta4_1 + theta3_1);
        theta2 = theta2_1 + theta2_2;
        %disp(theta2);
    else
        
        XY_3 = sqrt((XY)^2 + (Z0 - intercept)^2);
        %disp(XY_3);

        Hyp_XY_3 = sqrt((XY_3)^2 + (a4)^2);
        %disp(Hyp_XY_3);
        
        theta4 = -90;
        %disp(theta4);

        theta2_1 = double((acos((XY)/(XY_3)) * (180/pi)));
        %disp(theta2_1);

        theta2_2 = double((acos((XY_3)/(Hyp_XY_3)) * (180/pi)));
        %disp(theta2_1);

        theta2_3 = acos(((Hyp_XY_3)^2 + (a2)^2 - (a3)^2) / (2 * Hyp_XY_3 * a2));
        theta2_3 = double((theta2_3) * (180/pi));
        %disp(theta2_3)

        theta2 = theta2_1 + theta2_2 + theta2_3;
        %disp(theta2);

        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        
        theta3 = -(180 - theta3_1);
        %disp(theta3);
        
    end
end

elseif(Z0 <= 4)
    
    XY = sqrt((X0)^2 + (Y0)^2);
    %disp(XY);

    XY_3 = (XY) - a4;
    %disp(XY_3);

    Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z0)^2);
    %disp(Hyp_XY_3);

    theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
    %disp(theta4_Trap);

    theta2_1 = 180 - theta4_Trap;
    %disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    %disp(theta4_1)

    theta4 = (theta4_1 + theta2_1);
    %disp(theta4);

    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));

    if (theta3_1 >= 90)
        theta2_2 = 180 - (theta3_1 + theta4_1);
        theta2 = theta2_2 - theta2_1;
        if (isneg(theta2) == 1)
            theta2 = 0;
        end
       
        %disp(theta2);
        theta3 = -(180 - theta3_1);
        %disp(theta3);
    else
    
        XY_3 = (XY);
        %disp(XY_3);
    
        Z3 = Z0 + a4;

        Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z3)^2);
        %disp(Hyp_XY_3);

        theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
        %disp(theta4_Tri);

        theta2_1 = 180 - (theta4_Tri + 90);
        %disp(theta2_1);

        theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
        theta4_1 = double((theta4_1) * (180/pi));
        %disp(theta4_1)

        theta4 = -(theta4_Tri - theta4_1);
        %disp(theta4);
        
        if theta4 >= -90
            theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
            theta3_1 = double((theta3_1) * (180/pi));
            %disp(theta3_1);
    
            theta3 = -(180 - theta3_1);
            %disp(theta3);
    
            theta2_2 = 180 - (theta4_1 + theta3_1);
            theta2 = theta2_2 - theta2_1;
            %disp(theta2);
        else
            
            XY_3 = sqrt((XY)^2 + (intercept - Z0)^2);
            %disp(XY_3);
            
            Hyp_XY_3 = sqrt((XY_3)^2 + (a4)^2);
            %disp(Hyp_XY_3);
        
            theta4 = -90;
            %disp(theta4);
    
            theta2_1 = double((asin((XY)/(XY_3)) * (180/pi)));
            %disp(theta2_1);

            theta2_2 = double((acos((XY_3)/(Hyp_XY_3)) * (180/pi)));
            %disp(theta2_1);

            theta2_3 = 90 - (theta2_1 + theta2_2);
            %disp(theta2_3);
        
            theta2_4 = acos(((Hyp_XY_3)^2 + (a2)^2 - (a3)^2) / (2 * Hyp_XY_3 * a2));
            theta2_4 = double((theta2_4) * (180/pi));
            %disp(theta2_4)

            theta2 = theta2_4 - theta2_3;
            %disp(theta2);

            theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
            theta3_1 = double((theta3_1) * (180/pi));
        
            theta3 = -(180 - theta3_1);
            %disp(theta3);
        end
    end
    


elseif (Z0 > 4) && (Z0 <= 11)
    XY = sqrt((X0)^2 + (Y0)^2);
    %disp(XY);
    
    XY_3 = (XY) - a4;
    %disp(XY_3);
    
    Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z0)^2);
    %disp(Hyp_XY_3);
    
    theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
    %disp(theta4_Trap);
    
    theta2_1 = 180 - theta4_Trap;
    %disp(theta2_1);
    
    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    %disp(theta4_1)

    theta4 = (theta4_1 + theta2_1);
    %disp(theta4);
    
    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));
    
    if (theta3_1 >= 90)
        theta3 = -(180 - theta3_1);
        theta2_2 = 180 - (theta3_1 + theta4_1);
        theta2 = theta2_2 - theta2_1;
        %disp(theta2);
        %disp(theta3);
    else
        
        XY_3 = (XY);
        %disp(XY_3);
        
        Z3 = Z0 + a4;
        
        Hyp_XY_3 = sqrt((XY_3)^2 + (Z3 - intercept)^2);
        %disp(Hyp_XY_3);
        
        theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
        %disp(theta4_Tri);
        
        theta2_1 = 180 - (theta4_Tri + 90);
        %disp(theta2_1);
        
        theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
        theta4_1 = double((theta4_1) * (180/pi));
        %disp(theta4_1)
        
        theta4 = -(180 - (theta4_Tri + theta4_1));
        %disp(theta4);
    
        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        %disp(theta3_1);
    
        theta3 = -(180 - theta3_1);
        %disp(theta3);
    
        theta2_2 = 180 - (theta4_1 + theta3_1);
        theta2 = theta2_2 - theta2_1;
        %disp(theta2);
    
    end
end



theta1 = double((atan(Y0/X0) * (180/pi)));
%disp(theta1);

elseif(isneg(X0) == 1)
    
    if (Z0 >= intercept)

XY = sqrt((X0)^2 + (Y0)^2);
%disp(XY);

XY_3 = (XY) - a4;
%disp(XY_3);

Hyp_XY_3 = sqrt((XY_3)^2 + (Z0 - intercept)^2);
%disp(Hyp_XY_3);

theta4_Trap = double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
%disp(theta4_Trap);

theta2_1 = 180 - theta4_Trap;
%disp(theta2_1);

theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
theta4_1 = double((theta4_1) * (180/pi));
%disp(theta4_1)

theta4 = - theta4_1;
%disp(theta4);

theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
theta3_1 = double((theta3_1) * (180/pi));

if (theta3_1 >= 90)
    theta3 = (180 - theta3_1);
    theta2_2 = 180 - (theta3_1 + theta4_1);
    theta2 = 180 - (theta2_1 + theta2_2);
    %disp(theta2);
    %disp(theta3);
else
    %Down picking
    %theta3 = -90;
    %theta2_2 = 180 - ((90-theta3_1) + theta4_1 + 90);
    %theta2 = theta2_1 + theta2_2;
    %%disp(theta2);
    
    XY_3 = (XY);
    %disp(XY_3);
    
    Z3 = Z0 + a4;

    Hyp_XY_3 = sqrt((XY_3)^2 + (Z3 - intercept)^2);
    %disp(Hyp_XY_3);

    theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
    %disp(theta4_Tri);

    theta2_1 = 180 - (theta4_Tri + 90);
    %disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    %disp(theta4_1)

    theta4 = (180-(theta4_1 + theta4_Tri));
    %disp(theta4);

    if theta4 >= -90
        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        %disp(theta3_1);
    
        theta3 = (180 - theta3_1);
        %disp(theta3);
    
        theta2_2 = 180 - (theta4_1 + theta3_1);
        theta2 = 180 - (theta2_1 + theta2_2);
        %disp(theta2);
    else
        
        XY_3 = sqrt((XY)^2 + (Z0 - intercept)^2);
        %disp(XY_3);

        Hyp_XY_3 = sqrt((XY_3)^2 + (a4)^2);
        %disp(Hyp_XY_3);
        
        theta4 = 90;
        %disp(theta4);

        theta2_1 = double((acos((XY)/(XY_3)) * (180/pi)));
        %disp(theta2_1);

        theta2_2 = double((acos((XY_3)/(Hyp_XY_3)) * (180/pi)));
        %disp(theta2_1);

        theta2_3 = acos(((Hyp_XY_3)^2 + (a2)^2 - (a3)^2) / (2 * Hyp_XY_3 * a2));
        theta2_3 = double((theta2_3) * (180/pi));
        %disp(theta2_3)

        theta2 = 180 - (theta2_1 + theta2_2 + theta2_3);
        %disp(theta2);

        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        
        theta3 = (180 - theta3_1);
        %disp(theta3);
        
    end
end

elseif(Z0 <= 4)
    
    XY = sqrt((X0)^2 + (Y0)^2);
    %disp(XY);

    XY_3 = (XY) - a4;
    %disp(XY_3);

    Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z0)^2);
    %disp(Hyp_XY_3);

    theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
    %disp(theta4_Trap);

    theta2_1 = 180 - theta4_Trap;
    %disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    %disp(theta4_1)

    theta4 = -(theta4_1 + theta2_1);
    %disp(theta4);

    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));

    if (theta3_1 >= 90)
        theta2_2 = 180 - (theta3_1 + theta4_1);
        theta2 = 180 - (theta2_2 - theta2_1);
        if (isneg(theta2) == 1)
            theta2 = 180;
        end
       
        %disp(theta2);
        theta3 = (180 - theta3_1);
        %disp(theta3);
    else
    
        XY_3 = (XY);
        %disp(XY_3);
    
        Z3 = Z0 + a4;

        Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z3)^2);
        %disp(Hyp_XY_3);

        theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
        %disp(theta4_Tri);

        theta2_1 = 180 - (theta4_Tri + 90);
        %disp(theta2_1);

        theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
        theta4_1 = double((theta4_1) * (180/pi));
        %disp(theta4_1)

        theta4 = (theta4_Tri - theta4_1);
        %disp(theta4);
        
        if theta4 <= 90
            theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
            theta3_1 = double((theta3_1) * (180/pi));
            %disp(theta3_1);
    
            theta3 = (180 - theta3_1);
            %disp(theta3);
    
            theta2_2 = 180 - (theta4_1 + theta3_1);
            theta2 = 180 - (theta2_2 - theta2_1);
            %disp(theta2);
        else
            
            XY_3 = sqrt((XY)^2 + (intercept - Z0)^2);
            %disp(XY_3);
            
            Hyp_XY_3 = sqrt((XY_3)^2 + (a4)^2);
            %disp(Hyp_XY_3);
        
            theta4 = 90;
            %disp(theta4);
    
            theta2_1 = double((asin((XY)/(XY_3)) * (180/pi)));
            %disp(theta2_1);

            theta2_2 = double((acos((XY_3)/(Hyp_XY_3)) * (180/pi)));
            %disp(theta2_1);

            theta2_3 = 90 - (theta2_1 + theta2_2);
            %disp(theta2_3);
        
            theta2_4 = acos(((Hyp_XY_3)^2 + (a2)^2 - (a3)^2) / (2 * Hyp_XY_3 * a2));
            theta2_4 = double((theta2_4) * (180/pi));
            %disp(theta2_4)

            theta2 = 180 - (theta2_4 - theta2_3);
            %disp(theta2);

            theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
            theta3_1 = double((theta3_1) * (180/pi));
        
            theta3 = (180 - theta3_1);
            %disp(theta3);
        end
    end
    
    elseif (Z0 > 4) && (Z0 <= 11)
        XY = sqrt((X0)^2 + (Y0)^2);
    %disp(XY);

    XY_3 = (XY) - a4;
    %disp(XY_3);

    Hyp_XY_3 = sqrt((XY_3)^2 + (intercept - Z0)^2);
    %disp(Hyp_XY_3);

    theta4_Trap =double(90 + (asin((XY_3)/(Hyp_XY_3)) * (180/pi)));
    %disp(theta4_Trap);

    theta2_1 = 180 - theta4_Trap;
    %disp(theta2_1);

    theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
    theta4_1 = double((theta4_1) * (180/pi));
    %disp(theta4_1)

    theta4 = -(theta4_1 + theta2_1);
    %disp(theta4);

    theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
    theta3_1 = double((theta3_1) * (180/pi));
    
    if (theta3_1 >= 90)
        theta3 = (180 - theta3_1);
        theta2_2 = 180 - (theta3_1 + theta4_1);
        theta2 = 180-(theta2_2 - theta2_1);
        %disp(theta2);
        %disp(theta3);
    else
        
        XY_3 = (XY);
        %disp(XY_3);
        
        Z3 = Z0 + a4;
        
        Hyp_XY_3 = sqrt((XY_3)^2 + (Z3 - intercept)^2);
        %disp(Hyp_XY_3);
        
        theta4_Tri = double(asin((XY_3)/(Hyp_XY_3)) * (180/pi));
        %disp(theta4_Tri);
        
        theta2_1 = 180 - (theta4_Tri + 90);
        %disp(theta2_1);
        
        theta4_1 = acos(((Hyp_XY_3)^2 + (a3)^2 - (a2)^2) / (2 * Hyp_XY_3 * a3));
        theta4_1 = double((theta4_1) * (180/pi));
        %disp(theta4_1)
        
        theta4 = -(180 - (theta4_Tri + theta4_1));
        %disp(theta4);
    
        theta3_1 = acos(((a3)^2 + (a2)^2 - (Hyp_XY_3)^2) / (2 * a3 * a2));
        theta3_1 = double((theta3_1) * (180/pi));
        %disp(theta3_1);
    
        theta3 = (180 - theta3_1);
        %disp(theta3);
    
        theta2_2 = 180 - (theta4_1 + theta3_1);
        theta2 = 180 - (theta2_2 - theta2_1);
        %disp(theta2);
    
    end
end
   


    theta1 = double((atan(Y0/X0) * (180/pi)));
    %disp(theta1);
    
end
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
    

    
    %Only changing the angles theta 2 and 3

        
%% Loop to iterate through the five angles asigned to the position array
    
%     term_init = 'a';


    angle_move = zeros(1,5);
    
    for motor_num = 1:5
        
        angle1 = mapfun(theta1, -90, 90, 0, 180);
        angle2 = mapfun(theta2, 0, 180, 180, 0);
        angle3 = mapfun(theta3, -90, 90, 180, 0);
        angle4 = mapfun(theta4, -90, 90, 180, 0);
             
        position_default = [angle1, angle2, angle3, angle4, 0];
        pos_line = position_default(motor_num);
        
       if (pos_line <= abs(181))
           
           angle_move(1,motor_num) = pos_line;
                  
       else
           
           angle_move(1,motor_num) = 180;
           disp("Invalid angle. Input value between 0 and 180");
           
       end
       
    end
    
    %implementation of the all angle string sending of data
    
    angle_out = int2str(angle_move(1)) + "b" + int2str(angle_move(2)) + "e" + int2str(angle_move(3))+ "w" ...
        + int2str(angle_move(4)) + "p" + int2str(angle_move(5)) + "j";
    pause(2.0);
    fprintf(ard, angle_out);  
    
    disp(angle_out);
%%     
%     X_gyro = fread(ard, 1, 'uint16');
%     Y_gyro = fread(ard, 1, 'uint16');
%     Z_gyro = fread(ard, 1, 'uint16');

%       X_gyro = fgets(ard);
      
%     Y_gyro = fscanf(ard);
%     Z_gyro = fscanf(ard);
    
    
%     disp(X_gyro);

%     format compact;
%     gyro_x = ['X: ', num2str(X_gyro)];
%     gyro_y = ['Y: ', num2str(Y_gyro)];
%     gyro_z = ['Z: ', num2str(Z_gyro)];
    
    
%     disp(gyro_x);
%     disp(gyro_y);
%     disp(gyro_z);

 %%       

%%Dislay section and closing the port

    out = ['X, Y, Z: ', num2str(X), ',' num2str(Y), ',' num2str(Z)];
    disp(out);    

    if ~isempty(instrfind)
         fclose(instrfind);
         delete(instrfind);
     end
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




