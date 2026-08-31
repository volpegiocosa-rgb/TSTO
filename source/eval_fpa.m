function fpa = eval_fpa(pos, vel)
	% eval_fpa  Flight path angle assoluto: angolo tra il vettore velocita'
	%           e il piano locale orizzontale (perpendicolare a pos).
	%
	% Input  : pos, vel  (3x1, m e m/s, frame In)
	% Output : fpa       (scalare, rad)

	r = norm(pos);
	v = norm(vel);
	if r > 1e-9 && v > 1e-9
		vr  = dot(pos, vel) / r;        % componente radiale della velocita'
		fpa = asin(max(-1, min(1, vr / v)));
	else
		fpa = 0;
	end
end
