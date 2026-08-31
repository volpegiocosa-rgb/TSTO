function dy = eom(t, y, other)
	% equation of motion in Octave for ode integration
	% it's the core of the simulation software
	%
	% Stato y (8 componenti):
	%   y(1:3) posizione ECEF/ECI [m]
	%   y(4:6) velocità          [m/s]
	%   y(7)   massa             [kg]
	%   y(8)   delta-v accumulato [m/s]

	dy = zeros(8,1);

	% 0. unroll other struct and y
	pos  = y(1:3);
	vel  = y(4:6);
	mass = y(7);
	dv   = y(8);                     %#ok<NASGU> % non usato: solo leggibilità

	rm   = norm(pos);

	ENV      = other.ENV;
	AER      = other.AER;
	MOT      = other.MOT;
	GUI      = other.GUI;
	MIS      = other.MIS;
	isignite = other.isignite;
	phase    = other.phase;          % fase presa da 'other' (serve a guidance)
	stage    = other.GUI.active_stage;

	% 1. gravity evaluation
	g = -ENV.mu / rm^3 * pos;

	% 2. atmospheric evaluation
	% Clamp ai bordi della tabella (0-100 km) per evitare NaN da
	% extrapolazione oltre l'ultima quota tabulata (il volo raggiunge
	% quote orbitali ben oltre 100 km nelle fasi 5-6): oltre il bordo si
	% mantengono i valori (gia' pressoché nulli a 100 km), approssimazione
	% documentata equivalente a "vuoto" per pressione/densita'.
	lla         = cart2geo(pos, ENV.wgs84);
	altitude_q  = min(max(lla(3), min(ENV.altitude)), max(ENV.altitude));
	atmospheric_density = interp1(ENV.altitude, ENV.atmospheric_density, altitude_q);
	sound_speed         = interp1(ENV.altitude, ENV.sound_speed, altitude_q);
	ambient_pressure    = interp1(ENV.altitude, ENV.ambient_pressure, altitude_q);

	% 3. aerodynamic properties (relative_speed, Mach: non dipendono dall'assetto)
	relative_speed = eval_relative_speed(pos, vel, ENV.omega_E); % velocità rispetto all'aria
	vrel           = norm(relative_speed);
	Mach           = vrel / sound_speed;

	% 4. guidance: u va calcolato PRIMA dell'AoA. 'other' e' passato per
	% valore ad ogni chiamata di eom.m dentro ode45 (nessuna variabile
	% globale, CLAUDE.md §4): un'AoA basata su un GUI.last_pitch "stale"
	% (aggiornato solo ai confini di fase da simulator.m) introdurrebbe un
	% errore sistematico sul Cd/Drag proprio nelle fasi a pitch rapido
	% (2,3). Si calcola quindi prima il comando di assetto istantaneo u
	% (analitico in funzione di t per i case 1,2,4,5,6; per il case 3
	% dipende invece da GUI.last_pitch/last_yaw, che li' rappresentano
	% correttamente l'assetto "congelato" all'inizio della fase 3, usato
	% una tantum per decidere il segno del pitch_rate) e poi si deriva
	% l'AoA vera, istantanea e auto-consistente, da u e relative_speed.
	u   = guidance(MIS, ENV, GUI, t, pos, vel, 0, relative_speed, phase);
	AoA = eval_AoA(u, relative_speed);   % angolo d'attacco totale, istantaneo

	if stage == 1
		% AER.Cd e' memorizzata come [righe=Mach, colonne=AoA_deg] (layout
		% di aero_ascent.csv): interp2(X,Y,Z,..) vuole Z di dimensione
		% [length(Y), length(X)], quindi X=AER.AoA, Y=AER.Mach. AoA e'
		% qui in radianti (eval_AoA/acos): va convertita in gradi per la
		% griglia AER.AoA (verificato: interface_specification.md §5 TODO).
		% Clamp ai bordi della tabella per evitare NaN da extrapolazione
		% (interp2 default e' NaN fuori griglia, CLAUDE.md §4/§9 vieta NaN
		% non gestiti): Cd tenuto costante oltre i bordi, approssimazione
		% documentata.
		Mach_q = min(max(Mach, min(AER.Mach)), max(AER.Mach));
		AoA_q  = min(max(AoA * 180/pi, min(AER.AoA)), max(AER.AoA));
		Cd = interp2(AER.AoA, AER.Mach, AER.Cd, AoA_q, Mach_q);
	else
		Cd             = 0; % fuori atmosfera:inutile perdere tempo
	end

	if vrel > 0.1
		Drag = 0.5 * atmospheric_density * Cd * AER.Sref * vrel^2 * vers(relative_speed);
	else
		Drag = zeros(3,1);
	end

	% 5. thrust properties
	if isignite
		mass_flow_rate = MOT(stage).mass_flow_rate * MOT(stage).number_of_ignite_engine;
		vacuum_thrust  = MOT(stage).vacuum_thrust * MOT(stage).number_of_ignite_engine;
		thrust         = vacuum_thrust - ambient_pressure * MOT(stage).nozzle_exit_area * MOT(stage).number_of_ignite_engine;
	else
		mass_flow_rate = 0;
		thrust         = 0;
	end

	vectorized_thrust = thrust * u;
	not_gravitational_acceleration = vectorized_thrust / mass - Drag / mass;

	% 6. derivatives
	dy(1:3) = vel;
	dy(4:6) = not_gravitational_acceleration + g;
	dy(7)   = -mass_flow_rate;
	dy(8)   = norm(not_gravitational_acceleration);
end
