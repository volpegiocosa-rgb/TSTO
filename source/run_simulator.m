% run_simulator  Script di lancio da CLI (CLAUDE.md §2):
%     octave --no-gui source/run_simulator.m
%
% simulator.m e' una function-file (RES = simulator(config)): Octave NON
% la invoca automaticamente se passata come file da riga di comando (solo
% la definisce). Questo script e' l'effettivo punto di ingresso CLI: usa
% un dataset di default, gira la simulazione e scrive l'output in /output
% (CLAUDE.md §2/§3). Per un run con dataset diverso, chiamare
% simulator(config) direttamente (es. da un altro script/test).

addpath(fileparts(mfilename('fullpath')));

if ~exist('config', 'var') || isempty(config)
	config = struct();
end
if ~isfield(config, 'input_dir') || isempty(config.input_dir)
	config.input_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'input', 'reference_LV');
end
if ~isfield(config, 'silent')
	config.silent = false;
end

RES = simulator(config);

output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'output');
write_output_csv(RES, output_dir);

printf('Simulazione completata: %d campioni, t_finale=%.2f s, quota_finale=%.1f km\n', ...
       numel(RES.theTimes), RES.theTimes(end), RES.theAltitude(end)/1000);
printf('Output scritto in: %s\n', output_dir);
