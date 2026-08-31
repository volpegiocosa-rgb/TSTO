function lla = cart2geo(pos, wgs84)
	% cart2geo  Conversione da cartesiane (ECEF/ECI-frozen-a-t0) a geodetiche
	%           su ellissoide WGS84, con il metodo iterativo di Bowring.
	%
	% NOTA: non riceve il tempo t, quindi NON corregge la rotazione terrestre
	% tra t0 e t (approssimazione accettata: errore trascurabile su altitude,
	% usato solo per le tabelle atmosferiche in eom.m; per longitudine/downrange
	% "veri" la correzione con ENV.omega_E*t va fatta a valle, in create_output.m).
	%
	% Input  : pos    (3x1, m)
	%          wgs84  ([Req, f])
	% Output : lla    (3x1: [lat; lon; alt], rad, rad, m)

	Req = wgs84(1);
	f   = wgs84(2);
	e2  = f * (2 - f);   % eccentricita' al quadrato

	x = pos(1);
	y = pos(2);
	z = pos(3);

	lon = atan2(y, x);
	p   = sqrt(x^2 + y^2);

	if p < 1e-9
		% punto (quasi) sull'asse polare: longitudine indefinita, si assume 0
		lat = sign(z) * pi/2;
		alt = abs(z) - Req * sqrt(1 - e2);
		lla = [lat; lon; alt];
		return
	end

	lat = atan2(z, p * (1 - e2));   % stima iniziale
	for iter = 1:10
		sinlat = sin(lat);
		N      = Req / sqrt(1 - e2 * sinlat^2);
		alt    = p / cos(lat) - N;
		lat_new = atan2(z, p * (1 - e2 * N / (N + alt)));
		if abs(lat_new - lat) < 1e-12
			lat = lat_new;
			break
		end
		lat = lat_new;
	end

	sinlat = sin(lat);
	N      = Req / sqrt(1 - e2 * sinlat^2);
	alt    = p / cos(lat) - N;

	lla = [lat; lon; alt];
end
