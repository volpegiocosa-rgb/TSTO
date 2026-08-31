function plotter(T, Y, RES, input_dir)
	% plotter  Genera i grafici richiesti da plot_list.md e li salva come
	%          PNG di alta qualita' (300 dpi) in /output/<dataset>, dove
	%          <dataset> e' il nome della cartella di input corrente (es.
	%          'reference_LV', 'validation_test': stessa cartella passata
	%          in config.input_dir a simulator.m). Chiamata da simulator.m
	%          solo se config.silent e' false/assente.
	%
	% Grafici "tipo 1" (plot_list.md): una grandezza per figure, grid on,
	% nome file = grandezza plottata (es. 'time_vs_altitude.png').
	% Grafico "tipo 2": pitch e yaw in subplot(2,1,*) con lo stesso range
	% dell'asse X per entrambi i subplot.
	%
	% Input : T, Y        (storia temporale grezza; non usata direttamente,
	%                      tutte le grandezze plottate sono gia' in RES)
	%         RES         (struct dei risultati, create_output.m)
	%         input_dir   (path alla cartella di input del dataset corrente,
	%                      usato solo per il nome della cartella di output)
	% Output: nessuno (crea figure e file PNG)

	dataset_name = local_dataset_name(input_dir);
	output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'output', dataset_name);
	if ~exist(output_dir, 'dir')
		mkdir(output_dir);
	end

	t = RES.theTimes;

	% --- Grafici tipo 1 --------------------------------------------------
	local_plot_single(t, RES.theAltitude / 1000,      't [s]', 'quota [km]',           'time_vs_altitude',        output_dir);
	local_plot_single(t, RES.thePitch * 180/pi,       't [s]', 'pitch [deg]',          'time_vs_pitch',           output_dir);
	local_plot_single(t, RES.theYaw * 180/pi,         't [s]', 'yaw [deg]',            'time_vs_yaw',             output_dir);
	local_plot_single(t, RES.theAOA * 180/pi,         't [s]', 'AoA [deg]',            'time_vs_AoA',             output_dir);
	local_plot_single(t, RES.theIncidence * 180/pi,   't [s]', 'incidence [deg]',      'time_vs_incidence',       output_dir);
	local_plot_single(t, RES.theSideslip * 180/pi,    't [s]', 'sideslip [deg]',       'time_vs_sideslip',        output_dir);
	local_plot_single(t, RES.theMass / 1000,          't [s]', 'massa [t]',            'time_vs_mass',            output_dir);
	local_plot_single(t, RES.theApogeeAltitude / 1000, 't [s]', 'quota apogeo [km]',   'time_vs_apogee_altitude', output_dir);
	local_plot_single(t, RES.theInclination * 180/pi, 't [s]', 'inclinazione [deg]',   'time_vs_inclination',     output_dir);
	local_plot_single(RES.theDWR / 1000, RES.theAltitude / 1000, ...
	                   'downrange [km]', 'quota [km]', 'downrange_vs_altitude', output_dir);

	% --- Grafico tipo 2: pitch e yaw, subplot(2,1,*), stesso range X -----
	fig = figure('Name', 'time_vs_pitch_yaw');
	t_range = [min(t), max(t)];
	subplot(2, 1, 1);
	plot(t, RES.thePitch * 180/pi); grid on;
	xlabel('t [s]'); ylabel('pitch [deg]'); xlim(t_range);
	subplot(2, 1, 2);
	plot(t, RES.theYaw * 180/pi); grid on;
	xlabel('t [s]'); ylabel('yaw [deg]'); xlim(t_range);
	local_save_png(fig, output_dir, 'time_vs_pitch_yaw');
end


function dataset_name = local_dataset_name(input_dir)
	% local_dataset_name  Nome della cartella foglia di input_dir, robusto
	%                     a un eventuale separatore finale.
	input_dir_clean = input_dir;
	if numel(input_dir_clean) > 1 && ismember(input_dir_clean(end), {'/', '\'})
		input_dir_clean(end) = [];
	end
	[~, dataset_name] = fileparts(input_dir_clean);
end


function local_plot_single(x, y, xlab, ylab, name, output_dir)
	% local_plot_single  Crea una figure singola (grid on) e la salva PNG.
	fig = figure('Name', name);
	plot(x, y);
	xlabel(xlab); ylabel(ylab); grid on;
	local_save_png(fig, output_dir, name);
end


function local_save_png(fig, output_dir, name)
	% local_save_png  Salva la figure come PNG di alta qualita' (300 dpi).
	set(fig, 'PaperPositionMode', 'auto');
	print(fig, fullfile(output_dir, [name, '.png']), '-dpng', '-r300');
end
