function [y,r,e]=conj_act(W,V,x,iterations)
eps1=1e-9; % 1e-6
eps2=1e-9; % 1e-3
[n,m]=size(W);
y=zeros(n,1); 
if nargin<4, iterations=150; end
for i=1:iterations
    r=V'*y;
    %subplot(4,1,2),plot(r),ylabel('r')
    e=x./(eps2+((r)));
    %subplot(4,1,3),plot(e),ylabel('e')
    y=(eps1+y).*(W*e);
    %subplot(4,1,4),plot(y),ylabel('y')
end	