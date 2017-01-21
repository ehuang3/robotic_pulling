function [ E ] = sym2exp( S )
%SYM2STR 
%   

%% Convert symbolic expression into string expression.

n = size(S,1);
m = size(S,2);

E = ['['];
for i = 1:n
    for j = 1:m
        exp = strtrim(evalc('disp(S(i,j))'));
        E = [E '(' exp ')'];
        if j == m
            break
        end
        E = [E ' '];
    end
    if i == n
        break
    end
    E = [E '; '];
end
E = [E '];'];

end

