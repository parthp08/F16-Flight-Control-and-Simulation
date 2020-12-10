function [u,v,w] = Valphabeta2uvw(V,alpha,beta)
% convert V, alpha, beta to u, v, w in body frame

u = V*cos(alpha)*cos(beta);
v = V*sin(beta);
w = V*sin(alpha)*cos(beta);

end