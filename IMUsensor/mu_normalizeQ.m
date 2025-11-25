function x = mu_normalizeQ(x)
% MU_NORMALIZEQ  Normalize the quaternion

  x(1:4)  = x(1:4) / norm(x(1:4));
  %x = x*sign(x(1));
end