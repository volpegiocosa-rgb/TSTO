function RES = simulator(config)
	% simulator  Fasatore della simulazione: gestisce una fase di volo dopo
	%            l'altra chiamando l'integratore ODE su eom.m.
	%            E' il punto di interfaccia con l'utente.
	%
	% Flusso:
	%   1) interface(input_dir) -> legge i CSV e costruisce 'other'
	%   2) fase 0 (lift-off) NON simulata: si calcola il propellente da bruciare
	%      per raggiungere il trigger (puo' risultare t = 0 s)
	%   3) loop sulle fasi 1..6 (con eventuale salto 1-4 -> 5 per esaurimento
	%      propellente anticipato), ciascuna interrotta da un event
	%   4) create_output.m -> RES ; plotter.m -> grafici
	%
	% Input  : config (struct con almeno il campo:
	%              config.input_dir  path alla cartella dei CSV di input
	%          e opzioni di run, es. config.tmax_phase, config.silent)
	% Output : RES (struct dei risultati; gli output seguono interface_specification.md)

	% -------------------------------------------------------------------
	% 1. Lettura input e mapping: CSV (convenzione codice) -> 'other'
	% -------------------------------------------------------------------
	other = interface(config.input_dir);

	if ~isfield(config, 'tmax_phase') || isempty(config.tmax_phase)
		% Limite superiore di tspan per fase: sufficiente a coprire la fase
		% piu' lunga (gravity turn, burn stadio1; insertion, fino a burn
		% completo stadio2) con margine. Costa poco: l'integrazione si
		% ferma comunque al primo event reale, questo e' solo il tetto di
		% sicurezza anti-runaway.
		config.tmax_phase = 1000;
	end

	% -------------------------------------------------------------------
	% 2. Fase 0 (Lift-off): NON simulata.
	%    Calcolo del propellente da bruciare per raggiungere il trigger di
	%    lift-off (accelerazione non gravitazionale = gravita' locale).
	%    Il risultato puo' essere t = 0 s.
	% -------------------------------------------------------------------
	y0_pad      = init_state(other);
	[t0, y0]    = eval_liftoff_propellant(other, y0_pad);

	% -------------------------------------------------------------------
	% 3. Loop di integrazione sulle fasi
	% -------------------------------------------------------------------
	T = [];
	Y = [];
	phase_track = [];   % fase (1..6) di appartenenza di ogni riga di T/Y

	t_start = t0;
	y_start = y0;
	phase   = 1;

	max_iterations = 12;   % rete di sicurezza anti-loop-infinito (6 fasi al piu' con 1 salto)
	iteration = 0;

	while phase <= 6
		iteration = iteration + 1;
		if iteration > max_iterations
			error('simulator:tooManyPhaseIterations', ...
			      'Numero di transizioni di fase eccessivo: possibile ciclo nella macchina a stati.');
		end

		% --- 3a. Aggiornamento STADIO ATTIVO (staging) ---------------
		% Stadio 1 per le fasi 1..4, stadio 2 dalle fasi 5..6.
		% La separazione del 1' stadio (+ fairing, rilasciata insieme,
		% CLAUDE.md/decisione utente) avviene UNA VOLTA, al primo ingresso
		% in fase 5, per esaurimento propellente stadio1 (trigger fase 4,
		% eventualmente anticipato dalle fasi 1-3).
		if phase <= 4
			other.GUI.active_stage = 1;
		else
			if other.GUI.active_stage == 1
				% primo ingresso in fase 5: separazione 1' stadio + fairing
				y_start(7) = y_start(7) - other.MASS.Minert1 - other.MASS.Mfairing;
			end
			other.GUI.active_stage = 2;
		end

		% --- 3b. Motore acceso/spento per la fase corrente -----------
		% Fasi propulse: 1,2,3,4,6 ; fase 5 (coasting) NON propulsa.
		other.isignite = (phase ~= 5);

		% --- 3c. Fase corrente passata a eom.m/guidance.m via 'other' -
		other.phase = phase;

		% --- 3d. Event handle specifico della fase -------------------
		event_fun = @(t, y) phase_event(t, y, other, phase);

		% --- 3e. Integrazione ODE a passo variabile ------------------
		ode_opts = odeset('Events', event_fun, 'RelTol', 1e-8, 'AbsTol', 1e-8);
		tspan    = [t_start, t_start + config.tmax_phase];

		[t_ph, y_ph, te, ye, ie] = ode45(@(t, y) eom(t, y, other), ...
		                                 tspan, y_start, ode_opts); %#ok<ASGLU>

		if isempty(ie)
			error('simulator:noEventTriggered', ...
			      ['Fase %d: nessun event ha fermato l''integrazione entro ' ...
			       'tspan (t_start=%g, tmax_phase=%g). Trigger di fase ' ...
			       'mancato o config.tmax_phase insufficiente.'], ...
			      phase, t_start, config.tmax_phase);
		end
		fired = ie(1);   % in caso di eventi simultanei, si prende il primo

		% --- 3f. Accumulo risultati -----------------------------------
		T = [T; t_ph];                                   %#ok<AGROW>
		Y = [Y; y_ph];                                   %#ok<AGROW>
		phase_track = [phase_track; phase * ones(size(t_ph))]; %#ok<AGROW>

		t_start = t_ph(end);
		y_start = y_ph(end, :).';

		% --- 3g. Aggiornamento memoria di guida (ultimo assetto) -----
		% Serve al case 3 di guidance.m (GUI.last_pitch/last_yaw = assetto
		% comandato all'uscita dalla fase appena conclusa).
		relative_speed_end = eval_relative_speed(y_start(1:3), y_start(4:6), other.ENV.omega_E);
		u_end = guidance(other.MIS, other.ENV, other.GUI, t_start, ...
		                  y_start(1:3), y_start(4:6), 0, relative_speed_end, phase);
		[other.GUI.last_pitch, other.GUI.last_yaw] = vect2angleOl(other.GUI.InOl.' * u_end);

		% --- 3h. Decisione della fase successiva ----------------------
		% Mappa (fase corrente, indice evento scattato) -> azione, secondo
		% CLAUDE.md §5 (trigger di fase + "Altri trigger" globali).
		switch phase
			case {1, 2, 3}
				% eventi: [trigger_fase; quota=0; propellente1_esaurito]
				switch fired
					case 1
						phase = phase + 1;
					case 2
						break; % quota=0 -> stop
					case 3
						phase = 5;  % propellente1 esaurito in anticipo -> salta a fase 5
				end
			case 4
				% eventi: [propellente1_esaurito (trigger); quota=0]
				switch fired
					case 1
						phase = 5;
					case 2
						break;
				end
			case 5
				% eventi: [trigger_temporale; quota=0]
				switch fired
					case 1
						phase = 6;
					case 2
						break;
				end
			case 6
				% eventi: [apogeo_target; quota=0; propellente2_esaurito]
				% in ogni caso (successo, crash, propellente esaurito) la
				% missione termina qui.
				break;
		end
	end

	% -------------------------------------------------------------------
	% 4. Costruzione output e grafici
	% -------------------------------------------------------------------
	RES = create_output(T, Y, other, phase_track);

	if ~isfield(config, 'silent') || ~config.silent
		plotter(T, Y, RES);
	end
end
