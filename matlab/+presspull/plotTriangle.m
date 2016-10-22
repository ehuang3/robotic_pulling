function [ ] = plotTriangle( edges, com, sups )
%PLOTTRIANGLE 
%   

%% 
X = [0, edges(1), 0, 0]';
Y = [0, 0, edges(2), 0]';
plot(X,Y)
axis equal
xlim([-0.05, edges(1) + 0.05])
ylim([-0.05, edges(1) + 0.05])
hold on
for i = 1:size(sups,2)
    s = sups(:,i);
    plot(s(1), s(2), 'rx')
end
plot(com(1), com(2), 'ko')
hold off


end

