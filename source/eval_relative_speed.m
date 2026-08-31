function relative_speed = eval_relative_speed(pos, vel, omega_E)
	% eval_relative_speed  Velocita' del veicolo relativa all'aria, cioe'
	%                      la velocita' inerziale meno la velocita' di
	%                      trascinamento dovuta alla rotazione terrestre
	%                      (l'atmosfera co-ruota con la Terra).
	%
	% Input  : pos      (3x1, m, frame In)
	%          vel      (3x1, m/s, frame In)
	%          omega_E  (scalare, rad/s, velocita' di rotazione terrestre)
	% Output : relative_speed (3x1, m/s)

	omega_vec = [0; 0; omega_E];
	v_wind    = cross(omega_vec, pos);
	relative_speed = vel - v_wind;
end
