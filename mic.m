function [A,B] = mic(x,u,xDimension,uDimension)
% The model identified function is used to learn dynamics with history
% online
X = x(:,1:xDimension+uDimension-1);
X_p = x(:,2:xDimension+uDimension); 
U = u(:,1:xDimension+uDimension-1);
Omega = [X' U'];
X_T = X_p';
for i = 1:xDimension
    ab(:,i) = linsolve(Omega,X_T(:,i));
end
AB = ab';        
A = AB(:,1:xDimension);
B = AB(:,xDimension+1:xDimension+uDimension);
