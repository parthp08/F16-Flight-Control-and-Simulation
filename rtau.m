function [rt] = rtau(dp)
    
%{
Based on the work of Stanle Bak, rtau.py
%}
if(dp <= 25)
    rt = 1.0;
elseif(dp >= 50)
    rt = .1;
else
    rt = 1.9 - .036*dp;
end
end
