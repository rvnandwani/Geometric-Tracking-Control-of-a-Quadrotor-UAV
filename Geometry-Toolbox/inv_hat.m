function a = inv_hat(ahat)
a = zeros(3,1);
a(1,1) = -ahat(2,3);
a(2,1) = ahat(1,3);
a(3,1) = -ahat(1,2);
end