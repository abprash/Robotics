function [R] = myHarrisCorner2 (Ix,Iy,threshold)
    
    gx = [-1 0 1; -2 0 2; -1 0 1];
    gy = [1 2 1; 0 0 0 ;-1 -2 -1];
    [rows,columns]=size(Ix)
    disp(rows)
    disp(columns)
    new_image = zeros(rows,columns);
    smoothing_h = fspecial('Gaussian',[3,3],1);
    Ix2 = Ix .* Iy;
    Iy2 = Iy .* Iy;
    Ixy = Ix .* Iy;
    SIx2 = myImageFilter(Ix2,smoothing_h);
    SIy2 = myImageFilter(Iy2,smoothing_h);
    SIxy = myImageFilter(Ixy,smoothing_h);
    %new_image = zeros(rows,columns);
    k = 0.04;
    for(r=1:rows)
        for(c=1:columns)
            matrix_m = [SIx2(r,c) SIxy(r,c); SIxy(r,c) SIy2(r,c)];
            cornerness_measure = (matrix_m(1)*matrix_m(4) - matrix_m(2)*matrix_m(3)) - (k*(trace(matrix_m)^2));
            new_image(r,c) = cornerness_measure;
        end
    end
    %max(max(new_image))
    new_image = new_image > threshold;
    imshow(new_image)

    R = zeros(1,2);
    [temp_rows,temp_cols ] = size(new_image);
    for(r1 = 1:temp_rows)
        for(c1=1:temp_cols)
            if new_image(r1,c1) == 1

                R = [R;[r1 c1]];
            end
        end
    end
end