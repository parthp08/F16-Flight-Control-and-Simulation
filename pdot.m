function [pd] = pdot(p3,p1)

%{
Based on the work of Stanle Bak, pdot.py
%}
if(p1 >= 50)
    if(p3 >= 50)
        t = 5;
        p2 = p1;
    else
        p2 = 60;
        t = rtau(p2 - p3);
    end
else
    if (p3 >= 50)
        t = 5;
        p2 = 40;
    else
        p2 = p1;
        t = rtau(p2 - p3);
    end
end

pd = t * (p2 - p3);

end
