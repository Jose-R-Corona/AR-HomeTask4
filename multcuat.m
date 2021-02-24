function q = multcuat(q1, q2)
    eta1 = q1(1);
    eps1 = q1(2:end);
    eta2 = q2(1);
    eps2 = q2(2:end);
    skew_eps1=[0,-eps1(3),eps1(2);eps1(3),0,-eps1(1);-eps1(2),eps1(1),0];
    q = [eta1*eta2 - eps1'*eps2; eta1*eps2 + eta2*eps1 + skew_eps1*eps2];
end