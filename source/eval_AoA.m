function AoA = eval_AoA(u, relative_speed)
	% eval_AoA  Angolo d'attacco totale: angolo tra l'asse di spinta
	%           comandato (u, versore) e la velocita' relativa all'aria.
	%
	% Input  : u               (3x1, versore, frame In)
	%          relative_speed  (3x1, m/s, frame In)
	% Output : AoA             (scalare, rad)

	vr = norm(relative_speed);
	if vr > 1e-3
		cosAoA = dot(u, relative_speed) / vr;
		cosAoA = max(-1, min(1, cosAoA));   % clamp per sicurezza numerica
		AoA = acos(cosAoA);
	else
		AoA = 0;
	end
end
