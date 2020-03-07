function C = preAllocateVariabeCellArrayZeros(m,n,n_frame)
%#codegen
    assert(n>=0);
    assert(m>=0);
    % create a variable size cell array 
    C = cell(m,n);
    % C{:} = 0;
    for i=1:m
       for j=1:n
           C{j} = 0;
       end
    end
    coder.varsize('C', [ 1 n_frame ] );
end