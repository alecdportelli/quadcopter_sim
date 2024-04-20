% makeVaAlphaBeta.m: Create airspeed, angle-of-attack and sideslip from
%   a wind-relative airspeed vector
%
%   [Va alpha beta] = makeVaAlphaBeta(v_rel_b)
%      Inputs:
%        v_rel_b: 3-element vector representing wind-relative airspeed
%                 in body coordinates
%      Outputs:
%        Va:      Airspeed (scalar). (units are same as input vector)
%        alpha:   Angle-of-attack, radians
%        beta:    Sideslip, radians
%
function [Va, alpha, beta] = makeVaAlphaBeta(v_rel_b)

% Replace the following with appropriate code
Va = norm(v_rel_b);
if(Va == 0.0)
    alpha = 0; % radians
    beta = 0; % radians
else
    alpha = atan2(v_rel_b(3), v_rel_b(1));
    beta = asin(v_rel_b(2)/Va);
end

end