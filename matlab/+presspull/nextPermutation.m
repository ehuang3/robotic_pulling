function [ p, done ] = nextPermutation( p, r )
%NEXTPERMUTATION

%% 
done = true;
if all(p == r)
    return;
end
for i = length(p):-1:1
    if p(i) ~= r(i)
        break;
    end
end
p(i) = p(i) + 1;
for j = i+1:length(p)
    p(j) = 1;
end
done = false;

end