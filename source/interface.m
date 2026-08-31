function other = interface(input_dir)
	% interface  Legge i file CSV del dataset di input (convenzione CODICE)
	%            e costruisce la struct contenitore 'other' usata da eom.m.
	%
	% Chiamata come PRIMO passo da simulator.m, prima dell'integrazione.
	% I CSV sono delimitati da spazio, con header nella prima riga.
	% Il documento interface_specification.md riguarda SOLO gli output (RES).
	%
	% Input  : input_dir  (path alla cartella con i CSV, es. 'input/reference_LV')
	% Output : other       (struct con campi ENV, AER, MOT, GUI, MIS, + runtime)

	if nargin < 1 || isempty(input_dir)
		error('interface:noInput', 'Specificare la cartella dei CSV di input.');
	end

	other = struct();

	% -------------------------------------------------------------------
	% 1. Lettura dei file scalari (formato: name value unit note)
	% -------------------------------------------------------------------
	LV   = read_scalar_csv(fullfile(input_dir, 'LV.csv'));
	ENVc = read_scalar_csv(fullfile(input_dir, 'ENV.csv'));
	GUIc = read_scalar_csv(fullfile(input_dir, 'GUID.csv'));
	GVAR = read_scalar_csv(fullfile(input_dir, 'GUIDANCE_VARS.csv'));
	MISc = read_scalar_csv(fullfile(input_dir, 'MIS.csv'));

	% -------------------------------------------------------------------
	% 2. ENV (ambiente) -> other.ENV   [costanti WGS84 + sito di lancio]
	% -------------------------------------------------------------------
	other.ENV.Req     = ENVc.Req;
	other.ENV.Rpole   = ENVc.Rpole;
	other.ENV.f       = ENVc.f;
	other.ENV.omega_E = ENVc.omega_E;
	other.ENV.mu      = ENVc.mu;
	other.ENV.lat     = ENVc.lat;
	other.ENV.lon     = ENVc.lon;
	other.ENV.hpad    = ENVc.hpad;
	% Ellissoide WGS84 per cart2geo (usato in eom.m)
	other.ENV.wgs84   = [ENVc.Req, ENVc.f];

	% Tabelle atmosferiche (US Standard Atmosphere 1976)
	atmo = read_table_csv(fullfile(input_dir, 'atmosphere.csv'));
	other.ENV.altitude            = atmo.data(:, atmo.col.altitude);
	other.ENV.ambient_pressure    = atmo.data(:, atmo.col.PAtm);
	other.ENV.atmospheric_density = atmo.data(:, atmo.col.rho);
	other.ENV.sound_speed         = atmo.data(:, atmo.col.Vsound);

	% -------------------------------------------------------------------
	% 2b. MASS (masse veicolo) -> other.MASS
	%     LV.csv contiene M0/Mfairing/Minert*/MProp*, non mappati da
	%     nessun'altra struct della specifica ma necessari a init_state.m
	%     (massa iniziale), al fasatore in simulator.m (separazione 1'
	%     stadio) e a phase_event.m (trigger di esaurimento propellente).
	%     Aggiunta esplicita e documentata (dati gia' presenti in LV.csv,
	%     solo non ancora raccolti in 'other').
	% -------------------------------------------------------------------
	other.MASS.M0       = LV.M0;         % massa al liftoff, esclusa payload
	other.MASS.Mfairing = LV.Mfairing;
	other.MASS.Minert1  = LV.Minert1;
	other.MASS.Minert2  = LV.Minert2;
	other.MASS.MProp1   = LV.MProp1;
	other.MASS.MProp2   = LV.MProp2;
	% Massa payload: nessun campo dedicato nella specifica/nei CSV.
	% M0 (550000 kg) = Minert1+Minert2+MProp1+MProp2+Mfairing (548100+1900),
	% quindi si assume payload = 0 kg (default documentato, non nascosto).
	other.MASS.Mpayload = 0;

	% -------------------------------------------------------------------
	% 3. AER (aerodinamica) -> other.AER   [griglia 2D Cd(Mach, AoA)]
	% -------------------------------------------------------------------
	other.AER.Sref = LV.Sref;
	aero = read_grid_csv(fullfile(input_dir, 'aero_ascent.csv'));
	other.AER.Mach = aero.rows;   % vettore Mach (righe)
	other.AER.AoA  = aero.cols;   % vettore AoA [deg] (colonne)
	other.AER.Cd   = aero.data;   % matrice Cd(Mach, AoA) per interp2

	% -------------------------------------------------------------------
	% 4. MOT (propulsione) -> other.MOT   [array a 2 elementi]
	%    n_engine mappato su number_of_ignite_engine per stadio.
	% -------------------------------------------------------------------
	% Stadio 1
	other.MOT(1).vacuum_thrust           = LV.Thrust1;
	other.MOT(1).mass_flow_rate          = LV.MR1;
	other.MOT(1).nozzle_exit_area        = LV.Aexit1;
	other.MOT(1).number_of_ignite_engine = LV.n_engine1;
	% Stadio 2
	other.MOT(2).vacuum_thrust           = LV.Thrust2;
	other.MOT(2).mass_flow_rate          = LV.MR2;
	other.MOT(2).nozzle_exit_area        = LV.Aexit2;
	other.MOT(2).number_of_ignite_engine = LV.n_engine2;

	% -------------------------------------------------------------------
	% 5. GUI (guida) -> other.GUI   [azimut + variabili di guida]
	% -------------------------------------------------------------------
	other.GUI.launch_azimuth        = GUIc.AZ;
	other.GUI.timeHS_Sep_control    = GUIc.timeHS_Sep_control;
	other.GUI.flux_HS_Sep           = GUIc.flux_HS_Sep;
	% Variabili di guida (da GUIDANCE_VARS.csv)
	other.GUI.zkick                 = GVAR.zkick;
	other.GUI.pitch_over_starting   = GVAR.pitch_over_starting;
	other.GUI.pitch                 = [GVAR.pitch_c1, GVAR.pitch_c2];
	other.GUI.transition_starting   = GVAR.transition_starting;
	other.GUI.pitch_rate_transition = GVAR.pitch_rate_transition;
	other.GUI.pitch_at_transition   = GVAR.pitch_at_transition;
	other.GUI.insertion_starting    = GVAR.insertion_starting;
	other.GUI.AoA_rate              = GVAR.AoA_rate;
	other.GUI.plane_controller      = [GVAR.plane_controller_kp, ...
	                                   GVAR.plane_controller_kd, ...
	                                   GVAR.plane_controller_ki];
	% Stato di staging: inizia allo stadio 1; simulator.m lo porta a 2
	% a fine fase 4 (separazione 1' stadio per esaurimento propellente).
	other.GUI.active_stage          = 1;
	% Memoria assetto (aggiornata dal fasatore durante il volo)
	other.GUI.last_pitch            = 0;
	other.GUI.last_yaw              = GUIc.AZ;
	% Rotazione Ol -> In: Ol e' l'orizzonte locale ENU (X=Est,Y=Nord,Z=Up)
	% congelato a t0 alle coordinate del sito di lancio; In e' l'ECEF
	% congelato a t0 (CLAUDE.md §6). Le colonne di InOl sono i versori
	% Est/Nord/Up del sito espressi nel frame In (rotazione standard
	% ENU->ECEF da lat/lon; NON dipende da AZ, che entra invece nel
	% comando di guidance tramite setOl).
	lat = ENVc.lat;
	lon = ENVc.lon;
	e_hat = [-sin(lon);            cos(lon);            0      ];
	n_hat = [-sin(lat)*cos(lon);  -sin(lat)*sin(lon);   cos(lat)];
	u_hat = [ cos(lat)*cos(lon);   cos(lat)*sin(lon);   sin(lat)];
	other.GUI.InOl                  = [e_hat, n_hat, u_hat];

	% -------------------------------------------------------------------
	% 6. MIS (missione) -> other.MIS
	% -------------------------------------------------------------------
	other.MIS.apogee_altitude_target     = MISc.apogee_altitude_target;
	other.MIS.perigee_altitude_target    = MISc.perigee_altitude_target;
	other.MIS.target_orbital_inclination = MISc.target_orbital_inclination;

	% -------------------------------------------------------------------
	% 7. Campi di controllo runtime (aggiornati da simulator.m per fase)
	% -------------------------------------------------------------------
	other.isignite = false;
	other.phase    = 0;

end


% =====================================================================
% Helper locali
% =====================================================================

function s = read_scalar_csv(fname)
	% Legge un CSV 'name value [unit note]' delimitato da spazio.
	% Restituisce una struct: s.<name> = value (numerico).
	fid = fopen(fname, 'r');
	if fid < 0
		error('interface:fopen', 'Impossibile aprire %s', fname);
	end
	s = struct();
	header = fgetl(fid);   %#ok<NASGU> % scarta l'header
	line = fgetl(fid);
	while ischar(line)
		line = strtrim(line);
		if ~isempty(line)
			parts = strsplit(line);
			key = parts{1};
			val = str2double(parts{2});
			s.(key) = val;
		end
		line = fgetl(fid);
	end
	fclose(fid);
end


function t = read_table_csv(fname)
	% Legge una tabella numerica con header di nomi colonna (spazio).
	% Restituisce: t.data (matrice), t.col.<name> = indice colonna.
	fid = fopen(fname, 'r');
	if fid < 0
		error('interface:fopen', 'Impossibile aprire %s', fname);
	end
	header = strtrim(fgetl(fid));
	names  = strsplit(header);
	t.col = struct();
	for k = 1:numel(names)
		t.col.(names{k}) = k;
	end
	data = [];
	line = fgetl(fid);
	while ischar(line)
		line = strtrim(line);
		if ~isempty(line)
			row = str2double(strsplit(line));
			data = [data; row];   %#ok<AGROW>
		end
		line = fgetl(fid);
	end
	fclose(fid);
	t.data = data;
end


function g = read_grid_csv(fname)
	% Legge una griglia 2D: prima riga = 'label col1 col2 ...' (valori colonna),
	% prime colonne successive = valore riga + valori matrice.
	% Restituisce: g.rows (vettore), g.cols (vettore), g.data (matrice).
	fid = fopen(fname, 'r');
	if fid < 0
		error('interface:fopen', 'Impossibile aprire %s', fname);
	end
	header = strtrim(fgetl(fid));
	hparts = strsplit(header);
	% hparts{1} e' l'etichetta (es. Mach\AoA_deg); il resto sono i valori colonna
	g.cols = str2double(hparts(2:end));
	rows = [];
	data = [];
	line = fgetl(fid);
	while ischar(line)
		line = strtrim(line);
		if ~isempty(line)
			parts = str2double(strsplit(line));
			rows = [rows; parts(1)];       %#ok<AGROW>
			data = [data; parts(2:end)];   %#ok<AGROW>
		end
		line = fgetl(fid);
	end
	fclose(fid);
	g.rows = rows;
	g.data = data;
end
