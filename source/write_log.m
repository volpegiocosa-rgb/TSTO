function write_log(T, Y, RES, termination_reason, input_dir) %#ok<INUSL>
	% write_log  Genera la tabella 'tab.out' descritta da log_tab.md e la
	%            salva come TXT in /output/<nome_cartella_input_dir>
	%            (stessa cartella/convenzione di plotter.m). Chiamata da
	%            simulator.m solo se config.silent e' false/assente.
	%
	% Righe scritte (log_tab.md):
	%   - inizio e fine di ogni fase attraversata (da RES.theGuidFlag, che
	%     e' allineato 1:1 a T/Y ed e' equivalente al phase_track interno
	%     di simulator.m/create_output.m)
	%   - il trigger che ha fatto terminare la simulazione: NESSUNA
	%     deduzione/soglia qui, e' il messaggio esplicito 'termination_reason'
	%     che simulator.m costruisce nel proprio switch di fase (§3h),
	%     nel punto stesso in cui decide di uscire dal loop (break) in
	%     base all'indice 'fired' restituito da ode45/phase_event.m
	%   - la pressione dinamica massima (max q)
	%   - il primo raggiungimento di Mach 1 (se avvenuto)
	%
	% delta_v [m/s]: RES non contiene l'integrale di stato y(8) (integrale
	% di norm(acc_non_grav), non ricostruibile da RES). Si riporta invece
	% il delta-v netto RES.theDV_Prop - RES.theDV_drag (propulsivo meno
	% perdite da drag), gia' disponibile in RES (decisione utente).
	%
	% Input  : T, Y                 (storia temporale grezza; non usata
	%                               direttamente, tutte le grandezze
	%                               riportate sono gia' in RES)
	%          RES                  (struct dei risultati, create_output.m)
	%          termination_reason   (stringa, costruita da simulator.m nel
	%                               punto esatto in cui l'integrazione si
	%                               ferma in via definitiva)
	%          input_dir            (path alla cartella di input del dataset
	%                               corrente, usato solo per il nome della
	%                               cartella di output)
	% Output : nessuno (crea /output/<dataset>/tab.out)

	dataset_name = local_dataset_name(input_dir);
	output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'output', dataset_name);
	if ~exist(output_dir, 'dir')
		mkdir(output_dir);
	end

	delta_v = RES.theDV_Prop - RES.theDV_drag;

	% -------------------------------------------------------------------
	% 1. Eventi di inizio/fine fase: un segmento per ogni tratto contiguo
	%    di RES.theGuidFlag costante (l'ordine di attraversamento reale,
	%    non necessariamente 1..6 per intero: es. salto diretto a fase 5
	%    per esaurimento propellente anticipato, CLAUDE.md §5). Nomi
	%    evento <= 12 caratteri (log_tab.md): P<n>_START / P<n>_END.
	% -------------------------------------------------------------------
	N = numel(T);
	seg_start = 1;
	names = {};
	idxs  = [];
	for k = 2:N+1
		if k > N || RES.theGuidFlag(k) ~= RES.theGuidFlag(seg_start)
			phase_id = RES.theGuidFlag(seg_start);
			names{end+1} = sprintf('P%d_START', phase_id); %#ok<AGROW>
			idxs(end+1)  = seg_start; %#ok<AGROW>
			names{end+1} = sprintf('P%d_END', phase_id); %#ok<AGROW>
			idxs(end+1)  = k - 1; %#ok<AGROW>
			seg_start = k;
		end
	end

	% -------------------------------------------------------------------
	% 2. max q e Mach 1
	% -------------------------------------------------------------------
	[~, idx_maxq] = max(RES.thePdyn);
	names{end+1} = 'MAXQ';
	idxs(end+1)  = idx_maxq;

	idx_mach1 = find(RES.theMach(1:end-1) < 1 & RES.theMach(2:end) >= 1, 1, 'first');
	if ~isempty(idx_mach1)
		idx_mach1 = idx_mach1 + 1;   % indice del primo campione con Mach >= 1
		names{end+1} = 'MACH1';
		idxs(end+1)  = idx_mach1;
	end

	% -------------------------------------------------------------------
	% 3. Ordinamento cronologico degli eventi 1-2 (sort stabile su T)
	% -------------------------------------------------------------------
	[~, order] = sort(T(idxs));
	names = names(order);
	idxs  = idxs(order);

	% -------------------------------------------------------------------
	% 4. Scrittura tab.out: colonne a larghezza fissa (log_tab.md), header
	%    seguito da una riga di separazione '=' lunga quanto l'header.
	% -------------------------------------------------------------------
	cols = local_column_spec();

	header_parts = cell(1, numel(cols));
	for c = 1:numel(cols)
		if strcmp(cols{c}.type, 's')
			header_parts{c} = sprintf(sprintf('%%-%ds', cols{c}.width), cols{c}.label);
		else
			header_parts{c} = sprintf(sprintf('%%%ds', cols{c}.width), cols{c}.label);
		end
	end
	header_line = strjoin(header_parts, ' ');

	fmt_parts = cell(1, numel(cols));
	for c = 1:numel(cols)
		if strcmp(cols{c}.type, 's')
			fmt_parts{c} = sprintf('%%-%ds', cols{c}.width);
		else
			fmt_parts{c} = sprintf('%%%d.%df', cols{c}.width, cols{c}.dec);
		end
	end
	fmt = [strjoin(fmt_parts, ' '), '\n'];

	fname = fullfile(output_dir, 'tab.out');
	fid = fopen(fname, 'w');
	if fid < 0
		error('write_log:fopen', 'Impossibile scrivere %s', fname);
	end

	fprintf(fid, '%s\n', header_line);
	fprintf(fid, '%s\n', repmat('=', 1, numel(header_line)));

	for k = 1:numel(idxs)
		local_write_row(fid, fmt, names{k}, idxs(k), T, RES, delta_v);
	end
	local_write_row(fid, fmt, termination_reason, N, T, RES, delta_v);

	fclose(fid);
end


function local_write_row(fid, fmt, name, idx, T, RES, delta_v)
	% local_write_row  Scrive una riga della tabella per il campione 'idx'.
	fprintf(fid, fmt, name, T(idx), RES.theAltitude(idx) / 1000, ...
	        RES.theMass(idx) / 1000, RES.theVrel(idx), RES.theMach(idx), ...
	        RES.thePitch(idx) * 180/pi, RES.theYaw(idx) * 180/pi, ...
	        RES.theIncidence(idx) * 180/pi, RES.theApogeeAltitude(idx) / 1000, ...
	        RES.theInclination(idx) * 180/pi, delta_v(idx));
end


function cols = local_column_spec()
	% local_column_spec  Definizione delle colonne di tab.out (log_tab.md):
	%                    etichetta, larghezza fissa, tipo ('s'=stringa,
	%                    'f'=numerico) e decimali. Usata sia per l'header
	%                    sia per la format string dei dati, cosi' che
	%                    restino sempre allineati.
	cols = { ...
		struct('label', 'evento',     'width', 12, 'type', 's', 'dec', 0), ...
		struct('label', 'tempo_s',    'width', 9,  'type', 'f', 'dec', 3), ...
		struct('label', 'quota_km',   'width', 10, 'type', 'f', 'dec', 3), ...
		struct('label', 'massa_ton',  'width', 9,  'type', 'f', 'dec', 1), ...
		struct('label', 'vrel_ms',    'width', 10, 'type', 'f', 'dec', 1), ...
		struct('label', 'mach',       'width', 6,  'type', 'f', 'dec', 1), ...
		struct('label', 'pitch_deg',  'width', 9,  'type', 'f', 'dec', 1), ...
		struct('label', 'yaw_deg',    'width', 8,  'type', 'f', 'dec', 1), ...
		struct('label', 'incid_deg',  'width', 9,  'type', 'f', 'dec', 1), ...
		struct('label', 'apoalt_km',  'width', 10, 'type', 'f', 'dec', 1), ...
		struct('label', 'incl_deg',   'width', 8,  'type', 'f', 'dec', 2), ...
		struct('label', 'dv_ms',      'width', 8,  'type', 'f', 'dec', 1) ...
	};
end


function dataset_name = local_dataset_name(input_dir)
	% local_dataset_name  Nome della cartella foglia di input_dir, robusto
	%                     a un eventuale separatore finale (identico a
	%                     plotter.m: duplicato locale, nessuna dipendenza
	%                     incrociata tra file, CLAUDE.md §4).
	input_dir_clean = input_dir;
	if numel(input_dir_clean) > 1 && ismember(input_dir_clean(end), {'/', '\'})
		input_dir_clean(end) = [];
	end
	[~, dataset_name] = fileparts(input_dir_clean);
end
