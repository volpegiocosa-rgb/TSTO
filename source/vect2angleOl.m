function [pitch, yaw] = vect2angleOl(uOl)
	% vect2angleOl  Inversa di setOl: da un versore nel frame Ol (X=Est,
	%               Y=Nord, Z=Up) restituisce pitch (elevazione) e yaw
	%               (azimut vero da Nord, orario).
	%
	% Input  : uOl          (3x1, versore nel frame Ol)
	% Output : pitch, yaw   (scalari, rad)

	uOl = uOl(:);
	z   = max(-1, min(1, uOl(3)));   % clamp per sicurezza numerica su asin
	pitch = asin(z);
	yaw   = atan2(uOl(1), uOl(2));
end
