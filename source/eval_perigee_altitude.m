function perigee_altitude = eval_perigee_altitude(pos, vel, mu, Req)
	% eval_perigee_altitude  Quota di perigeo dell'orbita osculante corrente
	%                        (ellisse/iperbole kepleriana istantanea), da
	%                        energia specifica e momento angolare specifico.
	%                        Approssimazione sferica (Req), coerente con la
	%                        definizione di MIS.perigee_altitude_target.
	%                        Mirror di eval_apogee_altitude.m (usata da
	%                        create_output.m per RES.thePerigeeAltitude).
	%
	% Input  : pos, vel  (3x1, m e m/s, frame In)
	%          mu        (parametro gravitazionale terrestre, m^3/s^2)
	%          Req       (raggio equatoriale terrestre, m)
	% Output : perigee_altitude  (scalare, m)

	r = norm(pos);
	v = norm(vel);

	energy = v^2 / 2 - mu / r;
	a = -mu / (2 * energy);

	h = cross(pos, vel);
	e_vec = cross(vel, h) / mu - pos / r;
	e = norm(e_vec);

	perigee_radius = a * (1 - e);
	perigee_altitude = perigee_radius - Req;
end
