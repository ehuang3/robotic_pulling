function [ CP0 ] = getSpacedContactPoints( V, K, n_cp )
%GETSPACEDCONTACTPOINTS 
%   

%% Compute total length of the boundary.
n_V = length(K);
d = 0;
for i = 1:n_V
    A = V(:,K(i,1));
    B = V(:,K(i,2));
    d = d + norm(B-A);
end

% Generate list of contact points.
CP0 = zeros([2,n_cp]);
cp_step = d / n_cp;
i = 1;
j = 1;
k = 0;
b = 0;
while i <= n_cp
    A = V(:,K(j,1));
    B = V(:,K(j,2));
    t = (i*cp_step-b)/norm(B-A);
    assert(0 <= t);
    if t > 1 + 1e-7
        b = b + norm(B-A);
        j = j + 1;
        continue
    end
    cp = (1-t)*A + t*B;
    CP0(:,i) = cp;
    i = i + 1;
end

end

