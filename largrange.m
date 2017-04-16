function yh=lagrange(x,y,xh)
n = length(x);
m = length(xh);
x = x(:);
y = y(:);
xh = xh(:);
yh = zeros(m,1);
c1 = ones(1,n-1);
c2 = ones(m,1);
for i=1:n
    xp = x([1:i-1 i+1:n]);
    yh = yh + y(i) * prod((xh*c1-c2*xp')./(c2*(x(i)*c1-xp')),2);
end
end
