%% Definition of symbols
syms m Xu Xt Zw Zq Zs Zb Iy Mq Mw xs xb x1 x2 x3 x4 x5 x6
syms x1dot x2dot x3dot x4dot x5dot x6dot

x5dot = (Zs/Zq) + (Zb/Zq) - x5 - (Zw*x3/Zq) %- (x3dot*(m+Zw)/Zq)


