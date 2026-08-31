function y0 = init_state(other)
	% init_state  Stato iniziale del veicolo sul pad di lancio (fase 0,
	%             non simulata), prima del calcolo del propellente
	%             bruciato pre-lift-off (eval_liftoff_propellant.m).
	%
	% Posizione: dalle coordinate geodetiche del sito (ENV.lat/lon/hpad) al
	%            frame In (ECEF/ECI congelato a t0).
	% Velocita' : velocita' di co-rotazione con la Terra (il pad e' fermo
	%            rispetto al suolo, ma il frame In e' inerziale).
	% Massa     : M0 (esclusa payload) + Mpayload (assunta 0 kg, non
	%            fornita da nessun CSV: vedi nota in interface.m).
	% Delta-v   : 0.
	%
	% Input  : other  (struct ENV/AER/MOT/GUI/MIS/MASS)
	% Output : y0     (8x1: [pos; vel; mass; dv])

	ENV = other.ENV;

	pos0 = geo2cart(ENV.lat, ENV.lon, ENV.hpad, ENV.wgs84);
	vel0 = cross([0; 0; ENV.omega_E], pos0);
	mass0 = other.MASS.M0 + other.MASS.Mpayload;

	y0 = [pos0; vel0; mass0; 0];
end


function pos = geo2cart(lat, lon, alt, wgs84)
	% geo2cart  Inversa di cart2geo.m: da geodetiche (WGS84) a cartesiane.
	Req = wgs84(1);
	f   = wgs84(2);
	e2  = f * (2 - f);

	N = Req / sqrt(1 - e2 * sin(lat)^2);

	x = (N + alt) * cos(lat) * cos(lon);
	y = (N + alt) * cos(lat) * sin(lon);
	z = (N * (1 - e2) + alt) * sin(lat);

	pos = [x; y; z];
end
