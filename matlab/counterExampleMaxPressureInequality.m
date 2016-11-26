%% Counter example for the max power inequality.

% Pick a random twist.
t = 2 * (rand([3,1]) - 0.5);

% Pick random points in the unit circle.
N = 100;
P = rand([1,N]);
T = 2*pi*rand([1,N]);
R = [P .* cos(T); P .* sin(T)];

% Shift points in a random direction.
R(1,:) = R(1,:) + 2*(rand-0.5);
R(2,:) = R(2,:) + 2*(rand-0.5);

% Plot
% plot(R(1,:), R(2,:),'b.')
% axis([-1 1 -1 1])
% grid on

% Compute the dissipated power.


% Solve for a large p*.
