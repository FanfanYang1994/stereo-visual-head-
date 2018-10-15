function [Retina]=dilation(retina)
Retina=zeros(size(retina));
for i=1:128
    for j=1:128
        Retina(i,j)=retina(i,j)||retina(i-1,j)||retina(i+1,j)||retina(i,j+1)||retina(i,j-1);
    end
end