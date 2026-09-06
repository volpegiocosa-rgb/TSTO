function [f, g, h] = traj_problem(x, other, opts)
	% traj_problem  Wrapper superiore per l'ottimizzatore esterno (CMA-ES/
	%               ARCH, solver_project — sostituisce la Differential
	%               Evolution originariamente prevista, stessa interfaccia
	%               [f,g,h]). Sovrascrive le variabili di guida di 'other'
	%               con x, lancia simulator.m e riduce la traiettoria
	%               risultante a [f, g, h] tramite eval_fgh.m.
	%
	% Le costanti lette da CSV (LV/ENV/atmosphere/aero_ascent/GUID/MIS: tutto
	% cio' che NON e' sovrascritto da x) NON vengono rilette qui: vanno
	% caricate UNA SOLA VOLTA da un main.m con 'other = interface(input_dir)'
	% e passate in ingresso come secondo argomento. Rileggere i CSV ad ogni
	% chiamata (una per ogni individuo/generazione dell'ottimizzatore) sarebbe
	% I/O ripetuto e inutile, dato che quei dati non cambiano durante
	% l'ottimizzazione.
	%
	% Flusso: x, other (base) -> other_run (GUI/MASS sovrascritte) ->
	%         simulator.m (via config.other, NON config.input_dir: nessuna
	%         rilettura CSV) -> [RES, other_run] -> eval_fgh.m -> OPT -> f, g, h
	%
	% Input  : x      (10x1 o 1x10, variabili di design, in ordine fisso.
	%                   DECISIONE DI PROGETTO (conferma utente, sessione di
	%                   integrazione con solver_project): rispetto
	%                   all'insieme originario di GUIDANCE_VARS.csv (12
	%                   campi), plane_controller_kd/ki sono state RIMOSSE da
	%                   x (non hanno senso come variabili di design: restano
	%                   fisse al valore nominale letto da other.GUI, NON
	%                   sovrascritte qui) e sostituite dalla massa payload
	%                   (other.MASS.Mpayload, LV.csv). pitch_at_transition e'
	%                   stata SUCCESSIVAMENTE rimossa anch'essa (conferma
	%                   utente, sessione successiva): non e' un assetto
	%                   libero, e' vincolata a coincidere con l'assetto di
	%                   fine fase 2 -- ora derivata automaticamente in
	%                   simulator.m (sezione 3g/3h) da GUI.last_pitch, non
	%                   piu' sovrascritta qui da x:
	%                     1  zkick                  [m]
	%                     2  pitch_over_starting     [s]
	%                     3  pitch_c1                [rad/s^2]
	%                     4  pitch_c2                [rad/s]
	%                     5  transition_starting     [s]
	%                     6  pitch_rate_transition    [rad/s]
	%                     7  insertion_starting       [s]
	%                     8  AoA_rate                 [rad/s]
	%                     9  plane_controller_kp      [-]
	%                     10 Mpayload                 [kg]
	%          other  (struct, OBBLIGATORIA, output "pristine" di
	%                  interface.m: ENV/AER/MOT/GUI/MIS/MASS + runtime
	%                  isignite/phase, letti UNA VOLTA da un main.m e
	%                  costanti per tutta l'ottimizzazione. I campi
	%                  other.GUI/other.MASS corrispondenti a x vengono
	%                  sovrascritti qui su una COPIA locale — la 'other' del
	%                  chiamante non e' modificata, Octave passa gli struct
	%                  per valore — cosi' come i campi runtime
	%                  other.GUI.active_stage/last_pitch/last_yaw e
	%                  other.isignite/phase, per garantire che ogni chiamata
	%                  parta da uno stato "pre-lancio" pulito anche se
	%                  'other' venisse per errore passata dopo un run
	%                  precedente.)
	%          opts   (opzionale, struct con opzioni di risoluzione ODE, NON
	%                  costanti fisiche: opts.tmax_phase, opts.AbsTol,
	%                  opts.RelTol. Default = quelli di simulator.m.)
	%
	% Output : f  (scalare, da minimizzare)       = OPT.f = -Mpayload
	%          g  (vettore, vincoli g<=0)         = OPT.g
	%          h  (vettore 3x1, vincoli h=0)      = OPT.h
	%          (definizioni in eval_fgh.m / interface_specification.md §5.1)
	%
	% Punti x fisicamente non validi (nessun trigger di fase raggiunto entro
	% tmax_phase, loop di fase eccessivo, ecc.) NON propagano l'errore:
	% l'ottimizzatore esterno deve poter esplorare/scartare punti infeasible
	% senza arrestare l'intera ottimizzazione. In quel caso si restituisce
	% f = +Inf (peggiore possibile per un min(f)) e vincoli ampiamente
	% violati, cosi' il punto viene sempre respinto dalla selezione.

	x = x(:);
	if numel(x) ~= 10
		error('traj_problem:badInput', ...
		      ['x deve avere 10 componenti (variabili di design, vedi header ' ...
		       'traj_problem.m), ricevute %d.'], numel(x));
	end
	if nargin < 2 || isempty(other) || ~isfield(other, 'GUI')
		error('traj_problem:noOther', ...
		      ['other e'' obbligatoria: costruirla UNA VOLTA con other = ' ...
		       'interface(input_dir) in un main.m e passarla qui (vedi ' ...
		       'header traj_problem.m).']);
	end
	if nargin < 3 || isempty(opts)
		opts = struct();
	end

	% ---------------------------------------------------------------------
	% 1. other_run: copia locale di 'other' con le 10 variabili di design
	%    sovrascritte da x (altre costanti CSV invariate) e lo stato
	%    runtime pre-lancio reinizializzato (stesso stato prodotto da
	%    interface.m: nessuna dipendenza da run precedenti).
	%    pitch_at_transition NON viene sovrascritto da x (rimosso dalle
	%    variabili di design): resta al valore "pristine" di other finche'
	%    simulator.m non lo ricalcola da solo a fine fase 2 (vedi header).
	% ---------------------------------------------------------------------
	other_run = other;

	other_run.GUI.zkick                 = x(1);
	other_run.GUI.pitch_over_starting   = x(2);
	other_run.GUI.pitch                 = [x(3), x(4)];
	other_run.GUI.transition_starting   = x(5);
	other_run.GUI.pitch_rate_transition = x(6);
	other_run.GUI.insertion_starting    = x(7);
	other_run.GUI.AoA_rate              = x(8);
	% plane_controller_kd/ki (componenti 2,3) NON sovrascritti: restano al
	% valore nominale gia' in other.GUI.plane_controller (da GUIDANCE_VARS.csv
	% via interface.m) -- decisione di progetto sopra, non sono variabili di
	% design.
	other_run.GUI.plane_controller(1)   = x(9);
	other_run.MASS.Mpayload             = x(10);

	other_run.GUI.active_stage = 1;
	other_run.GUI.last_pitch   = 0;
	other_run.GUI.last_yaw     = other_run.GUI.launch_azimuth;
	other_run.isignite         = false;
	other_run.phase            = 0;

	% ---------------------------------------------------------------------
	% 2. config per simulator.m: 'other' gia' pronta (config.other), NESSUNA
	%    rilettura di CSV (config.input_dir non impostato).
	% ---------------------------------------------------------------------
	config = struct();
	config.other  = other_run;
	config.silent = true;   % nessun plot/log durante l'ottimizzazione
	% minimal_output: eval_fgh.m (unico consumatore di RES qui sotto) legge
	% solo theMass(end)/theApogeeAltitude(end)/thePerigeeAltitude(end)/
	% theInclination(end) -- create_output.m ricalcolerebbe l'intera storia
	% di reporting (stessa catena costosa di eom.m) per ogni punto, lavoro
	% sprecato durante l'ottimizzazione. Rif. CLAUDE.md solver_project S11
	% Fase 5, sessione "requisito 5 minuti" (vedi header simulator.m S4).
	config.minimal_output = true;
	if isfield(opts, 'tmax_phase') && ~isempty(opts.tmax_phase)
		config.tmax_phase = opts.tmax_phase;
	end
	% AbsTol/RelTol: non piu' usati da simulator.m (ode45 sostituito da
	% rk5.m, sessione fix-ode-hang) -- forwarding lasciato per non rompere
	% chiamanti esistenti che li passassero, ma config.AbsTol/RelTol non
	% sono letti da nessuno. tmin/tmax sono il loro sostituto (passo
	% min/max di rk5.m, vedi header simulator.m).
	if isfield(opts, 'AbsTol') && ~isempty(opts.AbsTol)
		config.AbsTol = opts.AbsTol;
	end
	if isfield(opts, 'RelTol') && ~isempty(opts.RelTol)
		config.RelTol = opts.RelTol;
	end
	if isfield(opts, 'tmin') && ~isempty(opts.tmin)
		config.tmin = opts.tmin;
	end
	if isfield(opts, 'tmax') && ~isempty(opts.tmax)
		config.tmax = opts.tmax;
	end

	% ---------------------------------------------------------------------
	% 3. simulator.m -> eval_fgh.m
	% ---------------------------------------------------------------------
	try
		[RES, other_final] = simulator(config);
		OPT = eval_fgh(RES, other_final);
	catch err %#ok<NASGU>
		OPT.f = Inf;
		OPT.g = [];
		OPT.h = 1e6 * ones(3, 1);
	end

	f = OPT.f;
	g = OPT.g;
	h = OPT.h;
end
