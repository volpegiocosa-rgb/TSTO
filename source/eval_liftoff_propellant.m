function [t0, y0] = eval_liftoff_propellant(other, y0_pad)
	% eval_liftoff_propellant  Fase 0 (lift-off, NON simulata, CLAUDE.md §5).
	%   Il veicolo e' fermo sul pad (posizione/quota costanti: g_locale e
	%   pressione ambiente costanti) mentre il 1' stadio spinge a regime
	%   fisso (nessun profilo di throttle modellato): la spinta e la
	%   portata sono quindi costanti nel tempo durante questa fase, e la
	%   massa cala linearmente. Il lift-off avviene quando la spinta a
	%   livello del mare eguaglia il peso (T/W = 1): essendo entrambe le
	%   grandezze lineari/costanti nel tempo, la massa al lift-off e il
	%   tempo t0 si ricavano in forma chiusa (nessuna iterazione).
	%
	% Se T/W >= 1 gia' a massa piena (come nel caso reference_LV, T/W
	% SL ~= 1.41), il propellente bruciato pre-lift-off e' nullo e t0 = 0
	% (coerente con la nota di CLAUDE.md §5).
	%
	% Input  : other   (struct ENV/AER/MOT/GUI/MIS/MASS)
	%          y0_pad  (8x1, stato a massa piena sul pad, da init_state.m)
	% Output : t0      (scalare, s, istante di lift-off)
	%          y0      (8x1, stato al lift-off: pos/vel invariati, massa
	%                   eventualmente ridotta, dv invariato a 0)

	ENV = other.ENV;
	MOT = other.MOT;

	pos0 = y0_pad(1:3);
	mass0 = y0_pad(7);

	ambient_pressure_pad = interp1(ENV.altitude, ENV.ambient_pressure, other.ENV.hpad);

	thrust_SL_total = (MOT(1).vacuum_thrust - ambient_pressure_pad * MOT(1).nozzle_exit_area) ...
	                  * MOT(1).number_of_ignite_engine;
	mass_flow_total = MOT(1).mass_flow_rate * MOT(1).number_of_ignite_engine;

	g_local = ENV.mu / norm(pos0)^2;

	mass_liftoff = thrust_SL_total / g_local;   % massa per cui T/W = 1

	if mass_liftoff >= mass0
		% T/W >= 1 gia' a massa piena: lift-off immediato, nessun burn.
		t0 = 0;
		y0 = y0_pad;
	else
		propellant_burned = mass0 - mass_liftoff;
		if propellant_burned > other.MASS.MProp1
			error('eval_liftoff_propellant:insufficientThrust', ...
			      ['Il 1 stadio non raggiunge T/W = 1 nemmeno esaurendo ' ...
			       'tutto MProp1: configurazione non fisicamente valida.']);
		end
		t0 = propellant_burned / mass_flow_total;
		y0 = y0_pad;
		y0(7) = mass_liftoff;
	end
end
