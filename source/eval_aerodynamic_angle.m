function [incidence, sideslip] = eval_aerodynamic_angle(relative_speed, pos, u_ref)
	% eval_aerodynamic_angle  Incidence (piano di pitch, "in-piano") e
	%                         sideslip (piano di yaw, "fuori piano") tra la
	%                         direzione di riferimento u_ref (versore, es.
	%                         assetto comandato) e la velocita' relativa,
	%                         decomposti nel frame Vn (VNC, CLAUDE.md §6)
	%                         costruito sulla velocita' relativa stessa:
	%                           X = vers(relative_speed)
	%                           Z = vers(pos x relative_speed)   (fuori dal
	%                               piano traiettoria-verticale locale)
	%                           Y = Z x X                        (nel piano
	%                               traiettoria-verticale, "conseguente")
	%
	% NON proiettare relative_speed e u_ref su un frame FISSO (es. Ol) per
	% poi sottrarre gli angoli di Eulero pitch/yaw calcolati separatamente:
	% la differenza di due angoli di Eulero non e' la decomposizione
	% ortogonale dell'angolo 3D tra i due vettori (accoppiamento
	% pitch/yaw), con bias crescente lontano da yaw costante e singolare
	% vicino alla verticale. Qui invece u_ref e' decomposto direttamente
	% nel frame ancorato al vento relativo: esente da bias per costruzione
	% (coerente con eval_AoA, che usa lo stesso prodotto scalare per
	% l'angolo totale: incidence/sideslip ne sono le componenti).
	%
	% Caso degenere pos parallelo a relative_speed (velocita' puramente
	% radiale, es. inizio verticale): il piano traiettoria-verticale non e'
	% definito, vers() restituisce il vettore nullo per Z e Y -> incidence
	% e sideslip si azzerano (scelta arbitraria ma stabile, non NaN).
	%
	% Input  : relative_speed  (3x1, m/s, frame In)
	%          pos             (3x1, m, frame In)
	%          u_ref           (3x1, versore, frame In)
	% Output : incidence, sideslip  (scalari, rad)

	x_hat = vers(relative_speed);
	z_hat = vers(cross(pos, relative_speed));
	y_hat = cross(z_hat, x_hat);

	u_Vn = [x_hat, y_hat, z_hat].' * u_ref;

	incidence = atan2(u_Vn(2), u_Vn(1));
	sideslip  = atan2(u_Vn(3), u_Vn(1));
end
