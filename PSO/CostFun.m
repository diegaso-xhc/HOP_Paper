function z = CostFun(x)
    
    % z = sum(x.^2);
%     z = (x-5).^2 + 4 + 100*sin(2*pi*0.1*x) - 40*sin(2*pi*2*x);
%     z = 5*(x^2) - (x-2).^2 + 4 - 10*sin(2*pi*0.1*x) - 200*sin(2*pi*0.2*x);
    
    % z = 10*x(1).^2 - 7*x(2).^2 + 1000*sin(2*pi*0.1*x(2));
    global t1 t2 t3 zr n
%     z = x(1)*t1.^2 + x(2)*t2.^2 + x(3)*t1.*t2;
%     z = (1/n)*sum(sum((z - zr).^2));
    
    z = x(1)*t1.^2 + x(2)*t2.^2 + x(3)*t1.*t2.*t3 + x(4)*t3.^3;
    z = (1/n)*sum(sum(sum((z - zr).^2)));
    
    
    
    
end