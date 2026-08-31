function incl = eval_inclination(pos, vel, ENV) %#ok<INUSD>
	% eval_inclination  Inclinazione orbitale istantanea (osculante), dal
	%                   vettore momento angolare specifico h = pos x vel,
	%                   rispetto all'asse Z (Polo Nord) del frame In.
	%
	% ENV non e' usato (l'asse Z del frame In e' gia' l'asse polare per
	% definizione, CLAUDE.md §6): mantenuto in firma per coerenza con
	% guidance.m / interface_specification.md.
	%
	% Input  : pos, vel  (3x1, m e m/s, frame In)
	%          ENV       (struct ambiente, non usata)
	% Output : incl      (scalare, rad, in [0, pi])

	h = cross(pos, vel);
	nh = norm(h);
	if nh > 1e-9
		incl = acos(max(-1, min(1, h(3) / nh)));
	else
		incl = 0;
	end
end
