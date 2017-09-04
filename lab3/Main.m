function [] = Main()
for i = 1: 5
    si = int2str(i);
    filename = strcat('img0',si,'.jpg');
    image = imread(filename);
    [r,c] = size(image);
    image1 = zeros(r,c);
    image1 = image;
    sigma = 1;
    threshold = 50000;
    if(ndims(image)>2)
        image = rgb2gray(image);
    end
    %call myImagefilter to get the smoothed 
    %smoothed_image = myImageFilter(image,[0.1096 0.1118 0.1096 ; 0.1118 0.1141 0.1118; 0.1096 0.1118 0.1096]);
    %perform edge detection
    [Im Io Ix Iy] = myEdgeFilter(image, sigma);
    [R] = myHarrisCorner2(Ix,Iy,threshold);
    %imshow(smoothed_image);
    a = figure;
    imshow(uint8(image1));
    %imshow(uint8(Ix));
    hold on;
    plot(R(:,2),R(:,1),'o')
    hold off;
    outputfile = strcat('outputimg00',si,'.jpg');
    saveas(a,outputfile);
end
end