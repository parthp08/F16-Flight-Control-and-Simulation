function value_sat = bound_function(value, lower_bound, upper_bound)
% keep the given parameter under the specified range
% and returns bounded value

if (value < lower_bound)
    value_sat = lower_bound;
elseif (value > upper_bound)
    value_sat = upper_bound;
else
    value_sat = value;
end

end