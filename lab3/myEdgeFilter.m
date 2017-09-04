function [Im Io Ix Iy ] = myEdgeFilter(img0, sigma)
    %sigma = 01;
    %gaussian matrix for smoothing
    smoothing_h = fspecial('Gaussian',3,sigma)
    image = myImageFilter(img0,smoothing_h);
    image = double(image);
    gx = [-1 0 1; -2 0 2; -1 0 1];
    gy = [1 2 1; 0 0 0 ;-1 -2 -1];
    [rows,columns]=size(image);
    new_image = zeros(rows,columns);
    Im = zeros(rows,columns);
    Ix =  zeros(rows,columns);
    Iy =  zeros(rows,columns);
    Io =  zeros(rows,columns);
    %imshow(new_image)
    temp_result = 0;
    [hrows,hcolumns] = size(gx);
    for(r=1:rows - 2)
        for(c=1:columns - 2)
            %we are now in the individual pixel
            current_value = image(r,c);
            temp_r = r;
            temp_c =c;
            val_x = -1*image(r,c) + image(r,c+2) -2*(image(r+1,c)) + 2*(image(r+1,c+2)) -1*(image(r+2,c)) +image(r+2,c+2);
            Ix(r,c) = val_x;
            val_y = image(r,c) + 2*(image(r,c+1)) + image(r,c+2) - image(r+2,c) - 2*image(r,c+1) - image(r,c+2) ;
            Iy(r,c) = val_y;
            g = sqrt(val_x^2 + val_y^2);
            new_image(r,c) = g;
            Io(r,c) = atan((val_x)/(val_y));
        end
    end
    %Ix = Ix;
    %Iy = Iy;
    %new_image = uint8(new_image);
    %imshow(uint8(Ix));
    %imshow(uint8(Iy));
    %Ix = uint8(Ix);
    %Iy = uint8(Iy);
    %imshow(Ix);
    %imshow(Iy);
    imshow(uint8(new_image));
    Im = new_image;
    %Io and Im are yet to be calculated
end