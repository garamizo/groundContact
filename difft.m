function yd = difft(y, var, vard)

syms t real
syms u1(t) u2(t) u3(t) u4(t) u5(t) u6(t) u7(t) u8(t) u9(t) u10(t) u11(t) u12(t) u13(t) u14(t) u15(t) u16(t) u17(t) u18(t) u19(t) 
u = {u1; u2; u3; u4; u5; u6; u7; u8; u9; u10; u11; u12; u13; u14; u15; u16; u17; u18; u19};
ud = cellfun(@diff, u, 'UniformOutput', false);

n = numel(var);

tmp = subs(y, var(:), u(1:n));
yd = diff(tmp, t);
yd = subs(yd, ud(1:n), vard(:));
yd = subs(yd, u(1:n), var(:));



