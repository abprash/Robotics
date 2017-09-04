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
disp(result);
disp(input_matrix);
end
