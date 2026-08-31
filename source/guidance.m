function uIn = guidance(MIS, ENV, GUI, t, pos, vel, AoA, relative_speed, phase)
	% guidance of the rocket/missile
	% it's based on sequential phases.
	%
	% Output:
	%   uIn : versore di spinta espresso nel frame inerziale

	switch phase
		case 1	% vertical rise
			uOl = [0; 0; 1];
			uIn = GUI.InOl * uOl;

		case 2  % pitch over
			% pitch(1)/pitch(2) sono coefficienti di uno scostamento dalla
			% verticale (dt^2/dt), non un pitch assoluto: senza l'offset
			% pi/2 la fase 2 partirebbe da elevazione 0 (orizzonte) invece
			% che dalla verticale con cui termina la fase 1 (uOl=[0;0;1]),
			% causando una discontinuita' di assetto. Coerente con
			% pitch_at_transition (~1.4 rad, fase 3), gia' un valore
			% assoluto vicino alla verticale.
			dt    = t - GUI.pitch_over_starting;
			pitch = pi/2 + GUI.pitch(1) * dt^2 + GUI.pitch(2) * dt;
			yaw   = GUI.launch_azimuth;
			uOl   = setOl(pitch, yaw);
			uIn   = GUI.InOl * uOl;

		case 3  % transition to gravity turn
			[incidence, ~] = eval_aerodynamic_angle(relative_speed, GUI.last_pitch, GUI.last_yaw, GUI.InOl);
			pitch_rate = -sign(incidence) * GUI.pitch_rate_transition;
			dt    = t - GUI.transition_starting;
			pitch = pitch_rate * dt + GUI.pitch_at_transition;
			yaw   = GUI.launch_azimuth;
			uOl   = setOl(pitch, yaw);
			uIn   = GUI.InOl * uOl;

		case {4,5} % gravity turn and coasting
			% relative_speed e' nel frame In: va ruotato in Ol (InOl.')
			% prima di poterne estrarre pitch/yaw con vect2angleOl, che si
			% aspetta componenti nel frame Ol (X=Est,Y=Nord,Z=Up).
			uOl_rel      = vers(GUI.InOl.' * relative_speed);
			[pitch, yaw] = vect2angleOl(uOl_rel);         % pitch seguito dalla velocità relativa
			yaw          = GUI.launch_azimuth;            % forced to follow the imposed azimuth
			uOl          = setOl(pitch, yaw);             % si mantiene il pitch calcolato, si forza lo yaw
			uIn          = GUI.InOl * uOl;

		case 6 % insertion in transfer orbit
			% Riferimento = direzione della velocita' relativa nel frame Ol
			% (come case {4,5}: relative_speed e' nel frame In, va ruotato
			% in Ol prima di estrarne pitch/yaw con vect2angleOl). NON
			% eval_fpa/setOl diretti: eval_fpa e' un FPA "locale" (rispetto
			% alla verticale del veicolo nella sua posizione corrente),
			% incompatibile se usato come pitch assoluto nel frame Ol
			% (ancorato al sito di lancio, congelato a t0): la discrepanza
			% cresce con la distanza downrange dal sito di lancio e
			% comandava un tuffo (pitch fino a -17deg) invece
			% dell'inserimento. Stesso principio per lo yaw: il PID di
			% piano corregge uno scostamento rispetto a yaw_rel (il
			% tracciamento nominale della velocita' relativa), non lo
			% sostituisce (altrimenti yaw diverge verso l'azimut zero
			% anziche' seguire la traiettoria, come osservato: -65deg).
			uOl_rel              = vers(GUI.InOl.' * relative_speed);
			[pitch_rel, yaw_rel] = vect2angleOl(uOl_rel);

			% yaw: tracciamento nominale + correzione controllore di piano
			actual_orbital_inclination = eval_inclination(pos, vel, ENV);
			target_orbital_inclination = MIS.target_orbital_inclination;
			error_orbital_inclination  = target_orbital_inclination ...
			                             - actual_orbital_inclination;
			kp = GUI.plane_controller(1);
			kd = GUI.plane_controller(2);
			ki = GUI.plane_controller(3);
			yaw = yaw_rel + PID_actuation(kp, kd, ki, error_orbital_inclination);

			% pitch: tracciamento nominale + AoA comandato crescente (per
			% l'inserimento, alza progressivamente il pitch sopra la
			% velocita' relativa)
			dt      = t - GUI.insertion_starting;
			AoA_cmd = GUI.AoA_rate * dt;
			pitch   = pitch_rel + AoA_cmd;

			% build the vector
			uOl = setOl(pitch, yaw);
			uIn = GUI.InOl * uOl;

		otherwise
			error('guidance:invalidPhase', ...
			      'Fase di guida non valida: %g', phase);
	end
end
