function [u,v,w] = Valphabeta2uvw(V,alpha,beta)
% convert V, alpha, beta to u, v, w in body frame

t_alpha = tan(alpha);
t_beta = tan(beta);

denomintor = sqrt(1 + (t_alpha*t_alpha) + (t_beta*t_beta));

u = V/denomintor;
v = V*t_beta/denomintor;
w = V*t_alpha/denomintor;
end