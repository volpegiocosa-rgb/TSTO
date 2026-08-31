function write_output_csv(RES, output_dir)
	% write_output_csv  Scrive RES in /output come CSV delimitati da spazio
	%                   (CLAUDE.md §2/§3), stesso formato dei CSV di input.
	%
	%   output_dir/RES.csv      : serie temporale (una riga per campione),
	%                              i campi vettoriali (thePOS_ECI, ecc.)
	%                              sono espansi in colonne _x/_y/_z.
	%   output_dir/summary.csv  : scalari fuori serie temporale
	%                              (funzione_costo).
	%
	% Input : RES         (struct, create_output.m)
	%         output_dir  (path alla cartella di output)

	if ~exist(output_dir, 'dir')
		mkdir(output_dir);
	end

	scalar_fields = {'theTimes', 'theMdot', 'theDWR', 'theX', 'theY', 'theZ', ...
	                  'theAltitude', 'theVx', 'theVy', 'theVz', ...
	                  'thePitch', 'theYaw', 'theAOA', 'theMach', 'thePdyn', ...
	                  'theFlux', 'theThrust', 'theDrag', 'theAcc', 'theAccProp', ...
	                  'theGuidFlag', 'theVrel', 'theFPARel', 'theFPA', ...
	                  'theLON', 'theLAT', 'theNGV', 'theDV_drag', 'theDV_Prop', ...
	                  'theMass', 'Flag_HSSep', 'Flag_MPLSep', 'stage'};
	vector_fields = {'thePOS_ECI', 'theVEL_ECI', 'theVREL_ECI'};

	N = numel(RES.theTimes);
	header_parts = {};
	columns      = {};

	for k = 1:numel(scalar_fields)
		name = scalar_fields{k};
		header_parts{end+1} = name; %#ok<AGROW>
		columns{end+1}      = double(RES.(name)(:)); %#ok<AGROW>
	end
	for k = 1:numel(vector_fields)
		name = vector_fields{k};
		suffix = {'_x', '_y', '_z'};
		for c = 1:3
			header_parts{end+1} = [name, suffix{c}]; %#ok<AGROW>
			columns{end+1}      = RES.(name)(:, c);  %#ok<AGROW>
		end
	end

	data = zeros(N, numel(columns));
	for k = 1:numel(columns)
		data(:, k) = columns{k};
	end

	fname = fullfile(output_dir, 'RES.csv');
	fid = fopen(fname, 'w');
	if fid < 0
		error('write_output_csv:fopen', 'Impossibile scrivere %s', fname);
	end
	fprintf(fid, '%s\n', strjoin(header_parts, ' '));
	fmt = [strjoin(repmat({'%.10g'}, 1, numel(columns)), ' '), '\n'];
	for r = 1:N
		fprintf(fid, fmt, data(r, :));
	end
	fclose(fid);

	fname_summary = fullfile(output_dir, 'summary.csv');
	fid = fopen(fname_summary, 'w');
	if fid < 0
		error('write_output_csv:fopen', 'Impossibile scrivere %s', fname_summary);
	end
	fprintf(fid, 'name value\n');
	fprintf(fid, 'funzione_costo %.10g\n', RES.funzione_costo);
	fclose(fid);
end
