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

% Triangle = imread('Triangle.jpg');
% Triangle = imread('IMG_20200224_000227.jpg');

%Triangle = imrotate(Triangle, -90);
subplot(2,3,1);
imshow(Triangle),
title('Original Image');

%Grayscale conversion
gra=rgb2gray(Triangle);
subplot(2,3,2);
imshow(gra),
title('Gray Image');

%Sharpening
gra = imsharpen(gra);

%Binarizing
%threshold = graythresh(gra);
threshold = 0.5;
BW = imbinarize(gra, threshold);
subplot(2,3,3);
imshow(BW),
title('Binary Image');

%Complementing
BW_inv = ~BW;
subplot(2,3,4);
imshow(BW_inv),
title('Inverted Binary Image');

%Filling the holes
Tri_fill = imfill(BW_inv, 'holes');
subplot(2,3,5);
imshow(Tri_fill),
title('Filled Image');

%masking to get back the grayscale image
Tri_mask = uint8(Tri_fill) .* gra;
subplot(2,3,6);
imshow(Tri_mask),
title('Masked Image');

%% Edge detection section

[Tri_canny, threshout] = edge(Tri_fill, 'canny', []);
figure, imshow(Tri_canny);
disp(threshout);

%For hough transform determination
[H,T,R] = hough(Tri_canny);
figure, imshow(H,[], 'XData', T, 'YData',R,...
    'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;

P = houghpeaks(H,10,'threshold', ceil(0.3*max(H(:))));
x = T(P(:,2));
y = R(P(:,1));
plot(x,y,'s', 'color', 'white');

lines = houghlines(Tri_canny,T,R,P,'FillGap',20,'MinLength',5);
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

matrix_XY_2 = [matrix_X(:,1), matrix_Y(:,1), matrix_X(:,2),matrix_Y(:,2)];

matrix_x_2 = [matrix_X(:,1); matrix_X(:,2)];
matrix_y_2 = [matrix_Y(:,1); matrix_Y(:,2)];

matrix_XY = [matrix_x_2 , matrix_y_2];
matrix_XY( ~any(matrix_XY,2),:) = [];

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
        matrix_XY2(Ind(ind,1),:) = []
    end
end

%Removing all-zeros columns
sort_XY( ~any(sort_XY,2),:) = [];

max_1 = max(sort_XY(:,1))
min_1 = min(sort_XY(:,1))
mid_1 = (min_1 + max_1)/2

curr_row_1 = 1;
curr_row_2 = 1;

sort_XY_2 = sort_XY;
for number = 1:30
    Ind_2 = find(sort_XY_2(:,2) == max(sort_XY_2(:,2)))
    indices_2 = length(Ind_2)
    
    for ind_2 = 1 : indices_2
        if (sort_XY_2(Ind_2((ind_2),1),1) > mid_1)
            XY_mat_1(curr_row_1, 1) = sort_XY_2(Ind_2((ind_2),1),1)
            XY_mat_1(curr_row_1, 2) = sort_XY_2(Ind_2((ind_2),1),2)
            curr_row_1 = curr_row_1 + 1
        
        else
            XY_mat_2(curr_row_2, 1) = sort_XY_2(Ind_2((ind_2),1),1)
            XY_mat_2(curr_row_2, 2) = sort_XY_2(Ind_2((ind_2),1),2)
            curr_row_2 = curr_row_2 + 1
        end
        
    end
    for ind_2 = indices_2:-1:1
        sort_XY_2(Ind_2(ind_2,1),:) = []
    end      
        
end