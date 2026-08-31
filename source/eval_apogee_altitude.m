function apogee_altitude = eval_apogee_altitude(pos, vel, mu, Req)
	% eval_apogee_altitude  Quota di apogeo dell'orbita osculante corrente
	%                       (ellisse/iperbole kepleriana istantanea), da
	%                       energia specifica e momento angolare specifico.
	%                       Approssimazione sferica (Req), coerente con la
	%                       definizione di MIS.apogee_altitude_target.
	%                       Estratta da phase_event.m (usata anche da
	%                       create_output.m per RES.theApogeeAltitude).
	%
	% Input  : pos, vel  (3x1, m e m/s, frame In)
	%          mu        (parametro gravitazionale terrestre, m^3/s^2)
	%          Req       (raggio equatoriale terrestre, m)
	% Output : apogee_altitude  (scalare, m)

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
