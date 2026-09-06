function OPT = eval_fgh(RES, other)
	% eval_fgh  Riduce l'esito di una simulazione a [f, g, h] per
	%           l'ottimizzatore esterno (CMA-ES/ARCH, solver_project), come
	%           da interface_specification.md §5.1.
	%
	% Firma adattata rispetto al placeholder di CLAUDE.md ("eval_fgh(T,Y)"),
	% come esplicitamente consentito da CLAUDE.md §7 ("possono essere
	% adattate"): si usa RES (gia' prodotta da create_output.m dentro
	% simulator.m, con l'ultima riga = ultimo istante di qualunque
	% condizione di arresto, propulsa o meno, interface_specification.md
	% §3.4) invece di ricalcolare da T,Y grezzi apogeo/perigeo/inclinazione:
	% quei valori sono gia' colonne di RES (theApogeeAltitude,
	% thePerigeeAltitude, theInclination), evitando di duplicare la logica
	% orbitale di create_output.m. 'other' serve per i target di missione
	% (other.MIS.*, non disponibili in RES) e per Mpayload (other.MASS.*,
	% decisione di progetto sotto).
	%
	% DECISIONE DI PROGETTO (rif. sessione di integrazione con solver_project,
	% conferma utente): la massa payload (other.MASS.Mpayload, LV.csv) e'
	% ora una VARIABILE DI DESIGN (11a componente di x, vedi traj_problem.m),
	% non piu' un valore fisso. L'obiettivo passa quindi dal proxy precedente
	% (massa residua a fine missione, -RES.theMass(end)) alla metrica di
	% performance diretta: f = -Mpayload (massimizzare il payload), con la
	% traiettoria che entra SOLO nei vincoli h (fattibilita' orbitale). Il
	% vecchio proxy era gia' dichiarato "se non valido, va sostituita qui"
	% nell'header precedente di questo file -- sostituzione ora eseguita.
	%
	% Input  : RES    (struct, output di create_output.m/simulator.m)
	%          other  (struct ENV/AER/MOT/GUI/MIS/MASS finale, secondo
	%                  output di simulator.m; qui si usano other.MIS e
	%                  other.MASS.Mpayload)
	% Output : OPT.f  (scalare, da minimizzare)  = -other.MASS.Mpayload
	%                  (massa payload, variabile di design in x, non un
	%                  esito della simulazione: dipende solo da 'other',
	%                  non da RES).
	%          OPT.g  (vettore, vincoli g<=0)     = [] (nessun vincolo di
	%                  disuguaglianza definito, placeholder come da
	%                  interface_specification.md §5.1)
	%          OPT.h  (vettore 3x1, vincoli h=0)  = errore residuo su
	%                  perigeo/apogeo/inclinazione rispetto al target di
	%                  missione (other.MIS), stesso ordine di
	%                  interface_specification.md §5.1:
	%                    (1) perigee_altitude_target  - perigee_achieved
	%                    (2) apogee_altitude_target   - apogee_achieved
	%                    (3) target_orbital_inclination - inclination_achieved

	if isempty(RES) || ~isfield(RES, 'theMass') || isempty(RES.theMass)
		error('eval_fgh:emptyRES', 'RES vuota o priva di theMass: nessun istante simulato da valutare.');
	end

	apogee_achieved      = RES.theApogeeAltitude(end);
	perigee_achieved     = RES.thePerigeeAltitude(end);
	inclination_achieved = RES.theInclination(end);

	OPT.f = -other.MASS.Mpayload;
	OPT.g = [];
	OPT.h = [other.MIS.perigee_altitude_target    - perigee_achieved; ...
	         other.MIS.apogee_altitude_target     - apogee_achieved; ...
	         other.MIS.target_orbital_inclination - inclination_achieved];
end
