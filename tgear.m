function [tg] = tgear(tht1)

%{
Based on the work of Stanle Bak, tgear.py
%}

    if (thtl <= .77)
        tg = 64.94 * thtl;
    else
        tg = 217.38 * thtl - 117.38;
    end
end
