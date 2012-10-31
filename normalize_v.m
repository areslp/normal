function [v] = normalize_v(v,a,b)
% 将v normalize 到[a,b]
maxVec=max(v);
minVec=min(v);
vecN=(v-minVec)./(maxVec-minVec);
v=vecN;
end
