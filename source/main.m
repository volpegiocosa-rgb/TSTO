% main  Punto di ingresso per l'ottimizzatore di traiettoria (CLAUDE.md §2):
%     octave --no-gui source/main.m
%
% Legge UNA SOLA VOLTA le costanti CSV del dataset (LV/ENV/atmosphere/
% aero_ascent/GUID/MIS: tutto cio' che NON e' una variabile di design x),
% le incapsula in 'other' tramite interface.m, e passa 'other' a
% traj_problem.m per ogni valutazione. Questo evita di rileggere i CSV da
% disco ad ogni chiamata di traj_problem (una per ogni individuo/
% generazione dell'ottimizzatore DE esterno che sostituira' il loop
% dimostrativo sotto).
%
% x0 e' qui inizializzato con gli stessi valori di GUIDANCE_VARS.csv del
% dataset (letti da other.GUI, gia' popolata da interface.m): serve solo a
% verificare il round-trip (stesso risultato del run diretto di
% simulator.m/eval_fgh.m), non e' un punto ottimizzato.

addpath(fileparts(mfilename('fullpath')));

if ~exist('config', 'var') || isempty(config)
	config = struct();
end
if ~isfield(config, 'input_dir') || isempty(config.input_dir)
	config.input_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'input', 'reference_LV');
end

% ---------------------------------------------------------------------
% 1. Costanti CSV -> 'other', UNA VOLTA SOLA (interface.m)
% ---------------------------------------------------------------------
other = interface(config.input_dir);

% ---------------------------------------------------------------------
% 2. x0: variabili di guida di baseline, lette da other.GUI (stesso
%    ordine di traj_problem.m / GUIDANCE_VARS.csv)
% ---------------------------------------------------------------------
x0 = [other.GUI.zkick; ...
      other.GUI.pitch_over_starting; ...
      other.GUI.pitch(1); ...
      other.GUI.pitch(2); ...
      other.GUI.transition_starting; ...
      other.GUI.pitch_rate_transition; ...
      other.GUI.pitch_at_transition; ...
      other.GUI.insertion_starting; ...
      other.GUI.AoA_rate; ...
      other.GUI.plane_controller(1); ...
      other.GUI.plane_controller(2); ...
      other.GUI.plane_controller(3)];

% ---------------------------------------------------------------------
% 3. Valutazione di traj_problem su x0 ('other' riutilizzabile per tutte
%    le successive chiamate dell'ottimizzatore DE, senza rileggere i CSV)
% ---------------------------------------------------------------------
[f, g, h] = traj_problem(x0, other);

printf('traj_problem(x0, other):\n');
printf('  f = %g\n', f);
printf('  g = ['); printf(' %g', g); printf(' ]\n');
printf('  h = ['); printf(' %g', h); printf(' ]\n');
