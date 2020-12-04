Triangle = imread('IMG_20200222_224458.jpg');
%Triangle = imrotate(Triangle, -90);
figure, imshow(Triangle),
title('Original Image');

gra=rgb2gray(Triangle);
figure, imshow(gra),
title('Gray Image');

threshold = graythresh(gra);
BW = imbinarize(gra, threshold);
figure, imshow(BW),
title('Binary Image');


BW = ~BW;
figure, imshow(BW),
title('Inverted Binary Image');

[B, L] = bwboundaries(BW, 'noholes');

[H,T,R] = hough(BW);
imshow(H,[], 'XData', T, 'YData',R,...
    'InitialMagnification','fit');
xlabel('\theta'), ylabel('rho');
axis on, axis normal, hold on;

P = houghpeaks(H,5,'threshold', ceil(0.3*max(H(:))));
x = T(P(:,2));
y = R(P(:,1));
plot(x,y,'s', 'color', 'white');

lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
figure, imshow(grayImage), hold on
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
end

STATS = regionprops(L, 'Centroid', 'Area', 'Perimeter');

%%
figure, imshow(Triangle),
title('Results');

hold on
for i = 1 : length(STATS)
    W(i) = uint8(abs(STATS(i).BoundingBox(3) - STATS(i).BoundingBox(4)) < 0.1);
    W(i) = W(i) + 2 * uint8((STATS(i).Extent - 1) == 0);
    centroid = STATS(i).Centroid;
    
    switch W(i)
        case 1
            plot(centroid(1),centroid(2), 'wO');
        case 2
            plot(centroid(1),centroid(2), 'wX');
        case 3
            plot(centroid(1),centroid(2), 'wS');
    end
end
