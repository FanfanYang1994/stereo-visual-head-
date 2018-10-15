function [W,V]=conjLearn(x,counter,W,V) 
% epsilon=1e-9;
% [~,m]=size(W);
%Vt=W./(epsilon+(max(W')'*ones(1,m)));%weights into nodes row-wise normalised 
% W(counter,:)=W(counter,:)+(e.*(Vt'*yc))'-W(counter,:);
% W(counter,:)=W(counter,:)./sum(W(counter,:));
W(counter,:)=x./sum(x);
V(counter,:)=W(counter,:)./max(W(counter,:));