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
			% yaw to reach target plane
			actual_orbital_inclination = eval_inclination(pos, vel, ENV);
			target_orbital_inclination = MIS.target_orbital_inclination;  
			error_orbital_inclination  = target_orbital_inclination ...
			                             - actual_orbital_inclination;      
			kp = GUI.plane_controller(1);
			kd = GUI.plane_controller(2);
			ki = GUI.plane_controller(3);
			yaw = PID_actuation(kp, kd, ki, error_orbital_inclination);
			% pitch
			flight_path_angle = eval_fpa(pos, vel);
			dt                = t - GUI.insertion_starting;
			AoA_cmd           = GUI.AoA_rate * dt;
			pitch             = flight_path_angle + AoA_cmd;
			% build the vector
			uOl = setOl(pitch, yaw);
			uIn = GUI.InOl * uOl;

		otherwise
			error('guidance:invalidPhase', ...
			      'Fase di guida non valida: %g', phase);
	end
end
