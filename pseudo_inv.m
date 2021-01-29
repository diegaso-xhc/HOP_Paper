function Jsi = pseudo_inv(J)
% This function calculates the pseudo inverse of an input matrix. Depending
% on the configuration of J, it follows two procedures. The first one for
% the case where J is tall (rows > columns) and the seconf one when J is
% fat (columns > rows)
m = size(J, 1); % No. of rows
n = size(J, 2); % No. of columns
if n > m
    tn = J*J';    
    Jsi = J'*(tn\eye(size(tn, 1)));
elseif n < m
    tn = J'*J;    
    Jsi = (tn\eye(size(tn, 1)))*J';
else
   Jsi = J\eye(n); % Simply the inverse of J
end
end