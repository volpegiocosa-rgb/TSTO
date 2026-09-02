function [f, g, h] = traj_problem(x, other, opts)
	% traj_problem  Wrapper superiore per l'ottimizzatore esterno
	%               (Differential Evolution su GUIDANCE_VARS.csv,
	%               interface_specification.md §2.6/§5.1). Sovrascrive le
	%               variabili di guida di 'other' con x, lancia
	%               simulator.m e riduce la traiettoria risultante a
	%               [f, g, h] tramite eval_fgh.m.
	%
	% Le costanti lette da CSV (LV/ENV/atmosphere/aero_ascent/GUID/MIS: tutto
	% cio' che NON e' sovrascritto da x) NON vengono rilette qui: vanno
	% caricate UNA SOLA VOLTA da un main.m con 'other = interface(input_dir)'
	% e passate in ingresso come secondo argomento. Rileggere i CSV ad ogni
	% chiamata (una per ogni individuo/generazione della DE) sarebbe I/O
	% ripetuto e inutile, dato che quei dati non cambiano durante
	% l'ottimizzazione.
	%
	% Flusso: x, other (base) -> other_run (GUI sovrascritta) -> simulator.m
	%         (via config.other, NON config.input_dir: nessuna rilettura CSV)
	%         -> [RES, other_run] -> eval_fgh.m -> OPT -> f, g, h
	%
	% Input  : x      (12x1 o 1x12, variabili di guida normalmente
	%                   ottimizzate, in ordine fisso — stesso ordine di
	%                   GUIDANCE_VARS.csv, interface_specification.md §2.6):
	%                     1  zkick                  [m]
	%                     2  pitch_over_starting     [s]
	%                     3  pitch_c1                [rad/s^2]
	%                     4  pitch_c2                [rad/s]
	%                     5  transition_starting     [s]
	%                     6  pitch_rate_transition    [rad/s]
	%                     7  pitch_at_transition      [rad]
	%                     8  insertion_starting       [s]
	%                     9  AoA_rate                 [rad/s]
	%                     10 plane_controller_kp      [-]
	%                     11 plane_controller_kd      [-]
	%                     12 plane_controller_ki      [-]
	%          other  (struct, OBBLIGATORIA, output "pristine" di
	%                  interface.m: ENV/AER/MOT/GUI/MIS/MASS + runtime
	%                  isignite/phase, letti UNA VOLTA da un main.m e
	%                  costanti per tutta l'ottimizzazione. I campi
	%                  other.GUI corrispondenti a x vengono sovrascritti qui
	%                  su una COPIA locale — la 'other' del chiamante non e'
	%                  modificata, Octave passa gli struct per valore — cosi'
	%                  come i campi runtime other.GUI.active_stage/
	%                  last_pitch/last_yaw e other.isignite/phase, per
	%                  garantire che ogni chiamata parta da uno stato
	%                  "pre-lancio" pulito anche se 'other' venisse per
	%                  errore passata dopo un run precedente.)
	%          opts   (opzionale, struct con opzioni di risoluzione ODE, NON
	%                  costanti fisiche: opts.tmax_phase, opts.AbsTol,
	%                  opts.RelTol. Default = quelli di simulator.m.)
	%
	% Output : f  (scalare, da minimizzare)       = OPT.f
	%          g  (vettore, vincoli g<=0)         = OPT.g
	%          h  (vettore 3x1, vincoli h=0)      = OPT.h
	%          (definizioni in eval_fgh.m / interface_specification.md §5.1)
	%
	% Punti x fisicamente non validi (nessun trigger di fase raggiunto entro
	% tmax_phase, loop di fase eccessivo, ecc.) NON propagano l'errore:
	% un ottimizzatore DE deve poter esplorare/scartare punti infeasible
	% senza arrestare l'intera ottimizzazione. In quel caso si restituisce
	% f = +Inf (peggiore possibile per un min(f)) e vincoli ampiamente
	% violati, cosi' il punto viene sempre respinto dalla selezione.

	x = x(:);
	if numel(x) ~= 12
		error('traj_problem:badInput', ...
		      ['x deve avere 12 componenti (variabili di guida, vedi header ' ...
		       'traj_problem.m / GUIDANCE_VARS.csv), ricevute %d.'], numel(x));
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
	% 1. other_run: copia locale di 'other' con le 12 variabili di guida
	%    sovrascritte da x (altre costanti CSV invariate) e lo stato
	%    runtime pre-lancio reinizializzato (stesso stato prodotto da
	%    interface.m: nessuna dipendenza da run precedenti).
	% ---------------------------------------------------------------------
	other_run = other;

	other_run.GUI.zkick                 = x(1);
	other_run.GUI.pitch_over_starting   = x(2);
	other_run.GUI.pitch                 = [x(3), x(4)];
	other_run.GUI.transition_starting   = x(5);
	other_run.GUI.pitch_rate_transition = x(6);
	other_run.GUI.pitch_at_transition   = x(7);
	other_run.GUI.insertion_starting    = x(8);
	other_run.GUI.AoA_rate              = x(9);
	other_run.GUI.plane_controller      = [x(10), x(11), x(12)];

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
	if isfield(opts, 'tmax_phase') && ~isempty(opts.tmax_phase)
		config.tmax_phase = opts.tmax_phase;
	end
	if isfield(opts, 'AbsTol') && ~isempty(opts.AbsTol)
		config.AbsTol = opts.AbsTol;
	end
	if isfield(opts, 'RelTol') && ~isempty(opts.RelTol)
		config.RelTol = opts.RelTol;
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
