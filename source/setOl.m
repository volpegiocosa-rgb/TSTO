function uOl = setOl(pitch, yaw)
	% setOl  Costruisce il versore di un asse (es. asse di spinta) nel frame
	%        Ol (X=Est, Y=Nord, Z=Up) a partire da pitch e yaw.
	%
	% Convenzioni:
	%   pitch : elevazione sull'orizzonte locale [rad] (0 = orizzontale, pi/2 = zenit)
	%   yaw   : azimut vero, misurato da Nord, positivo verso Est (orario) [rad]
	%           (coerente con GUI.launch_azimuth/AZ: 90 deg = verso Est)
	%
	% Input  : pitch, yaw  (scalari, rad)
	% Output : uOl         (3x1, versore nel frame Ol)

	uOl = [cos(pitch) * sin(yaw); ...
	       cos(pitch) * cos(yaw); ...
	       sin(pitch)];
end
