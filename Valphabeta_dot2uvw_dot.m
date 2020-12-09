function [Vdot,alphadot,betadot] = Valphabeta_dot2uvw_dot(u,v,w,udot,vdot,wdot)
vt = sqrt(u^2 + v^2 + w^2);
cbta = cos(asin(v/vt));
dum = u^2 + w^2;
Vdot = (u * udot + v * vdot + w * wdot)/vt;
alphadot = (u * wdot-w * udot)/dum;
betadot = (vt * vdot-v * Vdot) * cbta/dum;

end