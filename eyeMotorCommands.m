function [Pan,Tilt,y1]=eyeMotorCommands(W,V,NumRetFilters,fovealGaussian,yd,gaussCentersX,gaussCentersY)
retresp=zeros(1,NumRetFilters);
retresp(fovealGaussian)=1;
xL=[retresp';zeros(size(gaussCentersX))';zeros(size(gaussCentersY))';yd];
[y1,reconst,~]=conj_act(W,V,xL);
X=reconst(size(retresp,2)+1:(size(xL,1)-size(yd,1)));
[Pan,Tilt]=gaussDecode(X,gaussCentersX',gaussCentersY');