function OPT = eval_fgh(RES, other)
	% eval_fgh  Riduce l'esito di una simulazione a [f, g, h] per
	%           l'ottimizzatore esterno (Differential Evolution su
	%           GUIDANCE_VARS.csv), come da interface_specification.md §5.1.
	%
	% Firma adattata rispetto al placeholder di CLAUDE.md ("eval_fgh(T,Y)"),
	% come esplicitamente consentito da CLAUDE.md §7 ("possono essere
	% adattate"): si usa RES (gia' prodotta da create_output.m dentro
	% simulator.m, con l'ultima riga = ultimo istante di qualunque
	% condizione di arresto, propulsa o meno, interface_specification.md
	% §3.4) invece di ricalcolare da T,Y grezzi apogeo/perigeo/inclinazione/
	% massa: quei valori sono gia' colonne di RES (theApogeeAltitude,
	% thePerigeeAltitude, theInclination, theMass), evitando di duplicare la
	% logica orbitale di create_output.m. 'other' serve solo per i target di
	% missione (other.MIS.*), non disponibili in RES (che contiene i soli
	% valori raggiunti, non i target).
	%
	% Input  : RES    (struct, output di create_output.m/simulator.m)
	%          other  (struct ENV/AER/MOT/GUI/MIS/MASS finale, secondo
	%                  output di simulator.m; qui si usa solo other.MIS)
	% Output : OPT.f  (scalare, da minimizzare)  = -massa finale del
	%                  veicolo all'ultimo istante simulato (RES.theMass(end)).
	%                  Assunzione (interface_specification.md §5.1, "-massa
	%                  PL"): a parita' di payload fisso (other.MASS.Mpayload,
	%                  non una variabile di design qui) e di orbita target,
	%                  massima massa residua a fine missione = massimo
	%                  propellente stadio 2 non consumato nel burn di
	%                  injection (fase 8) = massimo margine di massa
	%                  utile, surrogato di "massa payload" nell'ottica di
	%                  un successivo dimensionamento. Se dichiarata
	%                  esplicitamente non valida, va sostituita qui.
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

	mass_achieved        = RES.theMass(end);
	apogee_achieved      = RES.theApogeeAltitude(end);
	perigee_achieved     = RES.thePerigeeAltitude(end);
	inclination_achieved = RES.theInclination(end);

	OPT.f = -mass_achieved;
	OPT.g = [];
	OPT.h = [other.MIS.perigee_altitude_target    - perigee_achieved; ...
	         other.MIS.apogee_altitude_target     - apogee_achieved; ...
	         other.MIS.target_orbital_inclination - inclination_achieved];
end
