function [incidence, sideslip] = eval_aerodynamic_angle(relative_speed, last_pitch, last_yaw, InOl)
	% eval_aerodynamic_angle  Incidence (piano di pitch) e sideslip (piano di
	%                         yaw) tra la velocita' relativa e l'ultimo
	%                         assetto comandato (last_pitch/last_yaw).
	%                         Usata dalla fase 3 (transizione al gravity
	%                         turn) per decidere il segno del pitch_rate.
	%
	% Input  : relative_speed  (3x1, m/s, frame In)
	%          last_pitch      (scalare, rad)
	%          last_yaw        (scalare, rad)
	%          InOl            (3x3, rotazione Ol -> In)
	% Output : incidence, sideslip  (scalari, rad)

	OlIn    = InOl.';                 % In -> Ol (rotazione ortogonale: trasposta = inversa)
	vrel_Ol = OlIn * relative_speed;

	if norm(vrel_Ol) > 1e-3
		[pitch_rel, yaw_rel] = vect2angleOl(vers(vrel_Ol));
	else
		pitch_rel = last_pitch;
		yaw_rel   = last_yaw;
	end

	incidence = pitch_rel - last_pitch;
	sideslip  = yaw_rel   - last_yaw;
end
