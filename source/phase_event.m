function [value, isterminal, direction] = phase_event(t, y, other, phase) %#ok<INUSL>
	% phase_event  Event function per ode45: un evento per il trigger
	%              specifico della fase corrente (CLAUDE.md §5) piu' gli
	%              event di sicurezza globali pertinenti alla fase (quota
	%              zero sempre; esaurimento propellente stadio attivo).
	%
	% L'ordine delle componenti e' fisso per fase (vedi switch sotto):
	% simulator.m usa l'indice 'ie' restituito da ode45 per capire quale
	% condizione ha fermato l'integrazione e decidere l'azione (avanzamento
	% normale di fase, salto a fase 5, stop definitivo).
	%
	% Input  : t, y     (tempo e stato correnti, come in eom.m)
	%          other    (struct ENV/AER/MOT/GUI/MIS/MASS)
	%          phase    (fase di volo corrente, 1..6)
	% Output : value, isterminal, direction  (vettori, spec di ode45 'Events')

	pos  = y(1:3);
	mass = y(7);

	ENV = other.ENV;
	MASS = other.MASS;

	lla = cart2geo(pos, ENV.wgs84);
	altitude = lla(3);

	% massa "morta" (tutto cio' che non e' propellente stadio1) durante le
	% fasi 1-4: stadio2 pieno + inerti di entrambi gli stadi + fairing
	% (sganciata insieme allo stadio 1, non prima) + payload.
	dead_mass_stage1 = MASS.Minert1 + MASS.Minert2 + MASS.MProp2 ...
	                  + MASS.Mfairing + MASS.Mpayload;
	% massa "morta" in fase 6: solo inerte stadio2 + payload (stadio1 e
	% fairing gia' separati a fine fase 4).
	dead_mass_stage2 = MASS.Minert2 + MASS.Mpayload;

	altitude_zero_ev = altitude;   % isterminal, direction=-1 (decrescente verso 0)

	switch phase
		case 1   % vertical-rise: trigger = quota raggiunge GUI.zkick
			value      = [altitude - other.GUI.zkick; ...
			              altitude_zero_ev; ...
			              mass - dead_mass_stage1];
			isterminal = [1; 1; 1];
			direction  = [1; -1; -1];

		case 2   % pitch over: trigger temporale (fine = GUI.transition_starting)
			value      = [t - other.GUI.transition_starting; ...
			              altitude_zero_ev; ...
			              mass - dead_mass_stage1];
			isterminal = [1; 1; 1];
			direction  = [1; -1; -1];

		case 3   % transizione al gravity turn: trigger = incidence == 0
			relative_speed = eval_relative_speed(pos, y(4:6), ENV.omega_E);
			[incidence, ~] = eval_aerodynamic_angle(relative_speed, ...
			                  other.GUI.last_pitch, other.GUI.last_yaw, other.GUI.InOl);
			value      = [incidence; ...
			              altitude_zero_ev; ...
			              mass - dead_mass_stage1];
			isterminal = [1; 1; 1];
			% incidence attraversa lo zero venendo dal segno opposto al
			% pitch_rate impostato in guidance.m (case 3): non se ne
			% conosce a priori il verso di attraversamento -> direction 0
			% (rileva l'attraversamento in entrambi i sensi).
			direction  = [0; -1; -1];

		case 4   % gravity turn: trigger = esaurimento propellente stadio1
			value      = [mass - dead_mass_stage1; ...
			              altitude_zero_ev];
			isterminal = [1; 1];
			direction  = [-1; -1];

		case 5   % coasting: trigger temporale (fine = GUI.insertion_starting)
			value      = [t - other.GUI.insertion_starting; ...
			              altitude_zero_ev];
			isterminal = [1; 1];
			direction  = [1; -1];

		case 6   % insertion: trigger = apogeo osculante raggiunge il target
			vel = y(4:6);
			apogee_altitude = eval_apogee_altitude(pos, vel, ENV.mu, ENV.Req);
			value      = [apogee_altitude - other.MIS.apogee_altitude_target; ...
			              altitude_zero_ev; ...
			              mass - dead_mass_stage2];
			isterminal = [1; 1; 1];
			direction  = [1; -1; -1];

		otherwise
			error('phase_event:invalidPhase', 'Fase non valida: %g', phase);
	end
end


function apogee_altitude = eval_apogee_altitude(pos, vel, mu, Req)
	% eval_apogee_altitude  Quota di apogeo dell'orbita osculante corrente
	%                       (ellisse/iperbole kepleriana istantanea), da
	%                       energia specifica e momento angolare specifico.
	%                       Approssimazione sferica (Req), coerente con la
	%                       definizione di MIS.apogee_altitude_target.
	r = norm(pos);
	v = norm(vel);

	energy = v^2 / 2 - mu / r;
	a = -mu / (2 * energy);

	h = cross(pos, vel);
	e_vec = cross(vel, h) / mu - pos / r;
	e = norm(e_vec);

	apogee_radius = a * (1 + e);
	apogee_altitude = apogee_radius - Req;
end
