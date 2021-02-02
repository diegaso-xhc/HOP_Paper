function vopt = cost(x, G, V)
vopt = G*x';
vopt = (1/length(x))*sum((vopt - V).^2);
% vopt = 1*sum((vopt(1:3,1) - V(1:3,1)).^2) + ...
%     2*sum((vopt(4:6,1) - V(4:6,1)).^2);
end