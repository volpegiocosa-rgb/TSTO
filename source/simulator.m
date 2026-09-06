function [RES, other] = simulator(config)
	% simulator  Fasatore della simulazione: gestisce una fase di volo dopo
	%            l'altra chiamando l'integratore ODE su eom.m.
	%            E' il punto di interfaccia con l'utente.
	%
	% Flusso:
	%   1) interface(input_dir) -> legge i CSV e costruisce 'other'
	%   2) fase 0 (lift-off) NON simulata: si calcola il propellente da bruciare
	%      per raggiungere il trigger (puo' risultare t = 0 s)
	%   3) loop sulle fasi 1..8 (con eventuale salto 1-4 -> 5 per esaurimento
	%      propellente anticipato); fasi 1-6 interrotte da un event rk5.m,
	%      fasi 7-8 istantanee (nessuna integrazione, CLAUDE.md §5)
	%   4) create_output.m -> RES ; plotter.m -> grafici
	%
	% Input  : config (struct con:
	%              config.input_dir  path alla cartella dei CSV di input;
	%                                usato SOLO se config.other non e'
	%                                fornito (letto da interface.m)
	%              config.other      (opzionale) struct 'other' gia'
	%                                costruita (es. da un main.m che ha
	%                                chiamato interface.m una volta sola e
	%                                poi passa 'other' a piu' run, come
	%                                traj_problem.m nel loop dell'
	%                                ottimizzatore DE): se presente, ha
	%                                priorita' su config.input_dir e
	%                                interface.m NON viene richiamata
	%          e opzioni di run, es. config.tmax_phase, config.silent,
	%          config.tmin, config.tmax (passo min/max di rk5.m, sessione
	%          fix-ode-hang -- sostituiscono config.AbsTol/RelTol di ode45,
	%          rimossi))
	% Output : RES    (struct dei risultati; segue interface_specification.md)
	%          other  (opzionale, struct ENV/AER/MOT/GUI/MIS/MASS finale usata
	%                  dalla simulazione; secondo output per non rompere le
	%                  chiamate esistenti "RES = simulator(config)". Usata da
	%                  traj_problem.m/eval_fgh.m per accedere a other.MIS
	%                  (target di missione) e other.ENV (mu, Req) senza
	%                  rileggere i CSV una seconda volta.)

	% -------------------------------------------------------------------
	% 1. Lettura input e mapping: CSV (convenzione codice) -> 'other'
	%    (oppure 'other' gia' pronta, passata da config.other)
	% -------------------------------------------------------------------
	if isfield(config, 'other') && ~isempty(config.other)
		other = config.other;
	else
		other = interface(config.input_dir);
	end

	if ~isfield(config, 'tmax_phase') || isempty(config.tmax_phase)
		% Limite superiore di tspan per fase: sufficiente a coprire la fase
		% piu' lunga (gravity turn, burn stadio1; insertion, fino a burn
		% completo stadio2) con margine. Costa poco: l'integrazione si
		% ferma comunque al primo event reale, questo e' solo il tetto di
		% sicurezza anti-runaway.
		config.tmax_phase = 1000;
	end

	% NOTA (rif. CLAUDE.md solver_project S11 Fase 5, sessione fix-ode-hang):
	% i default AbsTol/RelTol per ode45 (calibrati con lo studio di
	% sensitivita' descritto nelle sessioni precedenti) sono stati RIMOSSI
	% insieme alla chiamata a ode45 -- rk5.m (passo cinematico, non
	% controllo d'errore locale) li sostituisce con tmin/tmax sotto.
	% traj_problem.m puo' ancora ricevere/forwardare opts.AbsTol/opts.RelTol
	% per compatibilita' con chiamate esistenti: se presenti finiscono in
	% config.AbsTol/config.RelTol mai letti qui, innocuo.
	if ~isfield(config, 'tmin') || isempty(config.tmin)
		% TODO: PROVVISORIO -- non calibrato su un run a convergenza, solo
		% su smoke-test (round-trip nominale + candidato che causava
		% l'hang di ore). Limita il costo PEGGIORE per fase a
		% (tmax_phase)/tmin passi: con tmax_phase=1000s di default, 0.05s
		% da' un tetto di 20000 passi/fase, sufficiente a chiudere in
		% pochi secondi anche il caso quasi-inerziale che prima si
		% bloccava per ore.
		config.tmin = 0.05;
	end
	if ~isfield(config, 'tmax') || isempty(config.tmax)
		% TODO: PROVVISORIO -- non calibrato. Passo massimo nei tratti
		% cinematicamente "calmi" (tau=|v|/|a| grande): compromesso tra
		% velocita' di calcolo e localizzazione degli eventi (rk5.m
		% interpola linearmente tra due passi accettati).
		config.tmax = 2;
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
	phase_track = [];   % fase (1..8) di appartenenza di ogni riga di T/Y

	t_start = t0;
	y_start = y0;
	phase   = 1;
	termination_reason = '';   % messaggio esplicito, valorizzato al break (§3h), passato a write_log.m

	max_iterations = 14;   % rete di sicurezza anti-loop-infinito (8 fasi al piu' con 1 salto)
	iteration = 0;

	while phase <= 8
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

		% --- 3b-3h: fasi 1-6 integrate via ode45; fasi 7-8 istantanee -
		% (CLAUDE.md §5: fasi 7-8 non hanno un event associato, sono un
		% singolo calcolo in forma chiusa, non un intervallo integrato).
		if phase <= 6

			% --- 3b. Motore acceso/spento per la fase corrente -----------
			% Fasi propulse: 1,2,3,4,6 ; fase 5 (coasting) NON propulsa.
			other.isignite = (phase ~= 5);

			% --- 3c. Fase corrente passata a eom.m/guidance.m via 'other' -
			other.phase = phase;

			% --- 3e. Integrazione con passo cinematico (rk5.m) -----------
			% SOSTITUISCE ode45 (decisione utente, sessione fix-ode-hang,
			% rif. CLAUDE.md S11 Fase 5). Storia: provato prima ode23s
			% (semi-implicito, unica alternativa stiff funzionante su
			% questa build Octave -- ode15s fallisce sempre per sundials
			% mancante, ode23t/ode113 non esistono senza odepkg) solo
			% sulla fase 3 sospetta; scartato perche' non risolveva
			% l'hang in generale e introduceva ~8x di overhead anche sui
			% casi senza sintomi di stallo (18s -> 146s su un caso
			% facile). La diagnosi via logging per-eval (real_case/
			% traj_cost.m) ha poi trovato la causa CONCRETA: un candidato
			% con pitch_rate_transition~0 rende la fase 3 quasi-inerziale
			% per una finestra simulata molto lunga -- non un problema di
			% rigidezza, ma di passo che un controllo d'errore a
			% tolleranza fissa non riesce a far crescere abbastanza.
			% rk5.m (RK5 esplicito, passo cinematico via kinematic_step.m,
			% clippato a [tmin,tmax]) limita il costo peggiore per fase a
			% un numero di passi FISSO (tmax_phase/tmin), indipendente
			% dalla fisica del candidato.
			%
			% Convenzioni di chiamata (rif. header rk5.m): f e' una
			% closure 2 argomenti (t,y), stessa forma gia' in uso con
			% ode45; eventFcn e stepFcn ricevono invece 'other' come
			% terzo argomento esplicito (rk5.m lo forwarda).
			%
			% f_fun/event_fun: eom_native/phase_event_native (kernel
			% Fortran compilato, native/eom_core.f90 + shim native/
			% eom_oct.cc, native/phase_event_oct.cc) se i file .oct sono
			% sul path, altrimenti fallback sulle .m interpretate (rif.
			% CLAUDE.md solver_project S11 Fase 5, sessione "requisito 5
			% minuti": misurato 412.9x di speedup su eom.m/guidance.m,
			% 44.9x su phase_event.m -- overhead dell'interprete Octave
			% eliminato, non un problema di carico numerico). I parametri
			% (scalars/InOl/tabelle ENV,AER) sono costanti per l'intera
			% fase: costruiti UNA VOLTA qui, non ad ogni chiamata dentro
			% rk5.m. other.eom_fast viene passata anche a
			% kinematic_step.m (vedi suo header) cosi' anche la sua
			% chiamata a eom (1/7 per punto accettato, le altre 6 sono
			% k1..k6 di rk5.m) usa il percorso veloce.
			if exist('eom_native', 'file') == 3
				[scalars, InOl, env_alt, env_rho, env_c, env_p, aer_mach, aer_aoa, aer_cd] = ...
					build_eom_native_params(other);
				f_fun = @(t, y) eom_native(t, y, scalars, InOl, env_alt, env_rho, ...
					env_c, env_p, aer_mach, aer_aoa, aer_cd);
				other.eom_fast = f_fun;
			else
				f_fun = @(t, y) eom(t, y, other);
				other.eom_fast = [];
			end
			if exist('phase_event_native', 'file') == 3 && exist('scalars', 'var')
				event_fun = @(t, y, other) phase_event_native(t, y, scalars, InOl, phase);
			else
				event_fun = @(t, y, other) phase_event(t, y, other, phase);
			end
			tend = t_start + config.tmax_phase;

			[t_ph, y_ph, te, ye, ie] = rk5(f_fun, t_start, config.tmin, ...
			                                config.tmax, tend, y_start, ...
			                                other, event_fun, @kinematic_step); %#ok<ASGLU>

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
			% Serve al case 3 di guidance.m: GUI.last_pitch/last_yaw = assetto
			% comandato al termine della fase appena conclusa (istante
			% t_start, stesso istante di inizio della fase successiva: e'
			% quindi davvero "l'ultimo timestep noto", non un valore
			% arbitrariamente vecchio). Va PERO' notato che, una volta letto
			% da guidance.m/phase_event.m dentro la fase successiva, resta
			% congelato per l'intera durata di quella fase (non puo' essere
			% aggiornato ad ogni passo interno di ode45: 'other' e' catturato
			% per valore nella closure, nessuna variabile globale, CLAUDE.md
			% §4). E' quindi un riferimento "quasi-statico" fissato una
			% tantum all'inizio fase, usato in case 3 solo per decidere il
			% segno (costante per tutta la fase) di pitch_rate. Non serve
			% oltre la fase 6: le fasi 7-8 non richiamano piu' case 3.
			relative_speed_end = eval_relative_speed(y_start(1:3), y_start(4:6), other.ENV.omega_E);
			u_end = guidance(other.MIS, other.ENV, other.GUI, t_start, ...
			                  y_start(1:3), y_start(4:6), 0, relative_speed_end, phase);
			[other.GUI.last_pitch, other.GUI.last_yaw] = vect2angleOl(other.GUI.InOl.' * u_end);

			% pitch_at_transition NON e' piu' una variabile di design libera
			% (rimossa dall'ottimizzazione, decisione utente -- rif.
			% solver_project/CLAUDE.md S11 Fase 5): si deriva qui,
			% automaticamente, come l'assetto di pitch effettivamente
			% raggiunto al termine della fase 2 (stesso last_pitch appena
			% calcolato sopra). Cosi' la fase 3 (case 3 di guidance.m)
			% riparte per costruzione in continuita' con dove la fase 2 si
			% e' davvero fermata, invece che da un valore libero scorrelato
			% che poteva eccedere il tetto fisico di pi/2 rad (bug aperto
			% osservato prima di questa modifica).
			if phase == 2
				other.GUI.pitch_at_transition = other.GUI.last_pitch;
			end

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
							termination_reason = 'END_CRASH';
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
							termination_reason = 'END_CRASH';
							break;
					end
				case 5
					% eventi: [trigger_temporale; quota=0]
					switch fired
						case 1
							phase = 6;
						case 2
							termination_reason = 'END_CRASH';
							break;
					end
				case 6
					% eventi: [apogeo_target; quota=0; propellente2_esaurito]
					% fired==1 (apogeo target raggiunto) NON e' piu' terminale
					% (CLAUDE.md §5, fasi 7-8): prosegue in fase 7 (coast
					% kepleriano all'apogeo) e fase 8 (burn di injection).
					% fired==2/3 restano terminali: senza apogeo target
					% raggiunto non c'e' orbita di trasferimento da rifinire.
					switch fired
						case 1
							phase = 7;
						case 2
							termination_reason = 'END_CRASH';
							break;
						case 3
							termination_reason = 'END_PROP2';
							break;
					end
			end

		elseif phase == 7
			% --- Fase 7: Keplerian transfer (istantanea) ------------------
			% Coast non propulso dal termine fase 6 all'apogeo osculante
			% dell'orbita di trasferimento: propagazione kepleriana in forma
			% chiusa (flight_to_apogee.m), nessun event/ode45 necessario.
			other.isignite = false;
			other.phase    = phase;

			[t_apo, y_apo] = flight_to_apogee(t_start, ...
			                                   [y_start(1:3); y_start(4:6)], other.ENV);

			y_start = [y_apo; y_start(7); y_start(8)];   % massa/dv invariati (non propulsa)
			t_start = t_apo;

			T = [T; t_start];                    %#ok<AGROW>
			Y = [Y; y_start.'];                  %#ok<AGROW>
			phase_track = [phase_track; phase];  %#ok<AGROW>

			phase = 8;

		elseif phase == 8
			% --- Fase 8: Injection in target orbit (istantanea) -----------
			% Burn impulsivo (durata nulla: posizione/tempo invariati) col
			% propellente residuo dello stadio 2, per centrare apogeo/
			% perigeo/inclinazione target (injection_target_orbit.m).
			other.isignite = true;
			other.phase    = phase;

			dead_mass_stage2     = other.MASS.Minert2 + other.MASS.Mpayload;
			available_propellant = y_start(7) - dead_mass_stage2;
			ue2 = other.MOT(2).vacuum_thrust / other.MOT(2).mass_flow_rate;

			[v_final, dv_delivered, residual_mass, ...
			 apogee_reached, perigee_reached, inclination_reached] = ...
			    injection_target_orbit(y_start(1:3), y_start(4:6), y_start(7), ...
			        available_propellant, other.MIS.apogee_altitude_target, ...
			        other.MIS.perigee_altitude_target, ...
			        other.MIS.target_orbital_inclination, other.ENV, ue2);

			y_start = [y_start(1:3); v_final; residual_mass; y_start(8) + dv_delivered];

			T = [T; t_start];                    %#ok<AGROW>
			Y = [Y; y_start.'];                  %#ok<AGROW>
			phase_track = [phase_track; phase];  %#ok<AGROW>

			% Successo/parziale deciso per tolleranza (soglie documentate:
			% 1 km in quota, 0.1 deg in inclinazione, stesso ordine di
			% grandezza delle altre soglie fisiche del file): il burn eroga
			% comunque tutto il delta-v disponibile anche quando insufficiente
			% a centrare esattamente il target (injection_target_orbit.m
			% satura, non fallisce): non e' un errore, va solo distinto nel
			% log/output.
			tol_alt  = 1e3;
			tol_incl = 0.1 * pi / 180;
			on_target = abs(apogee_reached - other.MIS.apogee_altitude_target) < tol_alt && ...
			            abs(perigee_reached - other.MIS.perigee_altitude_target) < tol_alt && ...
			            abs(inclination_reached - other.MIS.target_orbital_inclination) < tol_incl;
			if on_target
				termination_reason = 'END_INSERTION';
			else
				termination_reason = 'END_INSERTION_PARTIAL';
			end

			phase = 9;   % esce dal loop (while phase <= 8)
		end
	end

	% -------------------------------------------------------------------
	% 4. Costruzione output e grafici
	% -------------------------------------------------------------------
	% config.minimal_output (rif. CLAUDE.md solver_project S11 Fase 5,
	% sessione "requisito 5 minuti"): create_output.m ricalcola, in un
	% secondo loop interpretato su TUTTI i punti T/Y (cart2geo, 3x interp1,
	% guidance, eval_AoA, interp2 -- la STESSA catena di eom.m), l'intera
	% storia di reporting (RES.theX/theMach/theAOA/...), anche quando
	% config.silent=true evita solo plotter.m/write_log.m. Misurato:
	% dominava il tempo residuo DOPO aver portato eom.m/phase_event.m in
	% Fortran (7.47s->1.32s con solo eom_native, quasi invariato dopo
	% anche phase_event_native: 1.30s -- il collo di bottiglia si era
	% spostato qui, non era piu' l'integrazione). eval_fgh.m (unico
	% consumatore di RES durante l'ottimizzazione, via traj_problem.m)
	% legge PERO' solo 4 valori scalari, tutti all'ISTANTE FINALE:
	% RES.theMass(end), RES.theApogeeAltitude(end), RES.thePerigeeAltitude(end),
	% RES.theInclination(end) -- verificato leggendo eval_fgh.m, non assunto.
	% Con config.minimal_output=true si costruiscono SOLO questi 4 campi
	% (scalari, dall'ultima riga di Y, stesse formule di create_output.m:
	% eval_apogee_altitude/eval_perigee_altitude/eval_inclination), senza
	% richiamare create_output.m. Default FALSE (nessuna modifica per i
	% chiamanti esistenti, es. il round-trip di main.m -- che infatti la
	% attiva anch'esso passando per traj_problem.m, a riprova che i due
	% percorsi restituiscono lo stesso risultato, vedi verifica in sessione).
	if isfield(config, 'minimal_output') && config.minimal_output
		pos_end = Y(end, 1:3).';
		vel_end = Y(end, 4:6).';
		RES = struct();
		RES.theMass            = Y(:, 7);
		RES.theApogeeAltitude  = eval_apogee_altitude(pos_end, vel_end, other.ENV.mu, other.ENV.Req);
		RES.thePerigeeAltitude = eval_perigee_altitude(pos_end, vel_end, other.ENV.mu, other.ENV.Req);
		RES.theInclination     = eval_inclination(pos_end, vel_end, other.ENV);
	else
		RES = create_output(T, Y, other, phase_track);
	end

	if ~isfield(config, 'silent') || ~config.silent
		if ~isfield(config, 'minimal_output') || ~config.minimal_output
			plotter(T, Y, RES, config.input_dir);
			write_log(T, Y, RES, termination_reason, config.input_dir);
		end
	end
end
