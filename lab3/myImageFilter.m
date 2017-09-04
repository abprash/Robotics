function [img1] = myImageFilter (img0,h)
    %image = imread(img0);
    image = img0;
    image = double(image);
    %h = [0.1 0.1 0.1; 0.1 0.2 0.1 ; 0.1 0.1 0.1];
    %h = [0.0113 0.0838 0.0113; 0.0838 0.6193 0.0838 ; 0.0113 0.0838 0.0113];
    %h = [0.06 0.1 0.06; .1 .2 .1; 0.06 .1 .06];
    %h = [1 1 1; 1 1 1; 1 1 1];
    %h = [ 0.0113 .0838 .0113 ; 0.0838 0.6193 0.0838;  0.0113 0.0838 0.0113];
    % sigma =5  [0.1096 0.1118 0.1096 ; 0.1118 0.1141 0.1118; 0.1096 0.1118 0.1096];
    h = matrixNormalization(h);
    disp(h)
    %disp(image)
    [rows,columns,x]=size(image);
    %disp(height)
    %disp(width)
    [hrows,hcolumns,x] = size(h);
    h = rot90(h,2);
    img1 = zeros(rows,columns);
    new_image = zeros(rows,columns);
    %imshow(new_image)
    temp_result = 0;
    for r=1:rows
        for c=1:columns
            temp_r = r;
            temp_c = c;
            %
            for inner_r = 1:hrows %the rows in filter matrix h
                for inner_c = 1:hcolumns %the columns in filter matrix h
                    if(temp_c-1<1)
                        %go to last column
                        col_value = columns;
                    else
                        col_value = temp_c - 1;
                    end
                    if(temp_r-1<1)
                        %go to last row

                        row_value = rows;
                    else
                        row_value=temp_r-1;
                    end
                    if (temp_r+1>rows)
                        %go to first row
                        row_value = mod(temp_r+1,rows);

                    else
                        row_value = temp_r+1;
                    end
                    if(temp_c+1>columns)
                        col_value = mod(temp_c+1,columns);

                    else
                        col_value = temp_c+1;
                    end
                    temp_result = (temp_result + (image(row_value, col_value )* (h(inner_r,inner_c))));
                    temp_c=temp_c+1;    
                end
                temp_r=temp_r+1;
            end
           new_image(r,c) = temp_result;
           temp_result = 0;
        end
    end
    %new_image = uint8(new_image);
    %imshow(new_image)
    img1 = new_image;
    imshow(uint8(img1));
end

function [result] = matrixNormalization(input_matrix)
%get the size of matrix
[input_rows,input_columns] = size(input_matrix);
%if either the rows or columns are even, convert them to odd
if(mod(input_rows,2)==0)
    rows = input_rows+1;
else
    rows = input_rows;
end
if(mod(input_columns,2)==0)
    columns = input_columns+1;
else
    columns = input_columns;
end
result = zeros(rows,columns);
for i = 1:(input_rows)
    for j = 1:(input_columns)
        result(i,j) = input_matrix(i,j);
    end
end
%disp(result);
%disp(input_matrix);
end

