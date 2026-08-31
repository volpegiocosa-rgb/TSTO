function RES = create_output(T, Y, other, phase_track)
	% create_output  Costruisce la struct RES (interface_specification.md
	%                §3) a partire dalla storia temporale T,Y prodotta da
	%                simulator.m e dal vettore ausiliario phase_track (fase
	%                1..6 di appartenenza di ogni riga: necessario per
	%                ricostruire RES.theGuidFlag/RES.stage, non derivabile
	%                da T,Y da soli).
	%
	% Le grandezze derivate (Mach, Cd, spinta, drag, ecc.) non sono
	% salvate da eom.m durante l'integrazione: vengono ricalcolate qui,
	% riga per riga, con le stesse funzioni ausiliarie di eom.m/guidance.m,
	% usando other.GUI.active_stage/isignite/phase coerenti con la fase di
	% quella riga (other e' quello finale restituito da simulator.m: i
	% campi statici ENV/AER/MOT/GUI(param.)/MIS non cambiano, solo
	% phase/isignite/active_stage vengono ricostruiti per riga qui).
	%
	% Input  : T            (Nx1, s)
	%          Y            (Nx8)
	%          other        (struct ENV/AER/MOT/GUI/MIS/MASS, finale)
	%          phase_track  (Nx1, fase 1..6 di ogni riga)
	% Output : RES  (struct, campi come da interface_specification.md §3.1)

	N = numel(T);

	ENV = other.ENV;
	AER = other.AER;
	MOT = other.MOT;
	MIS = other.MIS;

	theTimes    = T;
	theMdot     = zeros(N, 1);
	theX        = Y(:, 1);
	theY        = Y(:, 2);
	theZ        = Y(:, 3);
	theAltitude = zeros(N, 1);
	theVx       = Y(:, 4);
	theVy       = Y(:, 5);
	theVz       = Y(:, 6);
	thePOS_ECI  = Y(:, 1:3);
	theVEL_ECI  = Y(:, 4:6);
	theVREL_ECI = zeros(N, 3);
	thePitch    = zeros(N, 1);
	theYaw      = zeros(N, 1);
	theAOA      = zeros(N, 1);
	theMach     = zeros(N, 1);
	thePdyn     = zeros(N, 1);
	theFlux     = zeros(N, 1);
	theThrust   = zeros(N, 1);
	theDrag     = zeros(N, 1);
	theAcc      = zeros(N, 1);
	theAccProp  = zeros(N, 1);
	theGuidFlag = zeros(N, 1);
	theVrel     = zeros(N, 1);
	theFPARel   = zeros(N, 1);
	theFPA      = zeros(N, 1);
	theLON      = zeros(N, 1);
	theLAT      = zeros(N, 1);
	theNGV      = zeros(N, 1);
	theDV_drag  = zeros(N, 1);
	theDV_Prop  = zeros(N, 1);
	theMass     = Y(:, 7);
	Flag_HSSep  = false(N, 1);
	Flag_MPLSep = false(N, 1);
	stage       = zeros(N, 1);

	dv_drag_acc = 0;   % integrali trapezoidali di theDV_drag/theDV_Prop
	dv_prop_acc = 0;

	for k = 1:N
		t    = T(k);
		pos  = Y(k, 1:3).';
		vel  = Y(k, 4:6).';
		mass = Y(k, 7);
		phase = phase_track(k);

		% fase -> theGuidFlag / stage / active_stage / isignite (CLAUDE.md
		% §5 "Tabella unica" / interface_specification.md §3.3)
		if phase <= 4
			active_stage = 1;
			row_stage    = 1;
		elseif phase == 5
			active_stage = 2;
			row_stage    = 20;
		else
			active_stage = 2;
			row_stage    = 2;
		end
		isignite_row = (phase ~= 5);

		lla = cart2geo(pos, ENV.wgs84);
		altitude_q = min(max(lla(3), min(ENV.altitude)), max(ENV.altitude));
		atmospheric_density = interp1(ENV.altitude, ENV.atmospheric_density, altitude_q);
		sound_speed         = interp1(ENV.altitude, ENV.sound_speed, altitude_q);
		ambient_pressure    = interp1(ENV.altitude, ENV.ambient_pressure, altitude_q);

		relative_speed = eval_relative_speed(pos, vel, ENV.omega_E);
		vrel = norm(relative_speed);
		Mach = vrel / sound_speed;

		GUI_row = other.GUI;
		u = guidance(MIS, ENV, GUI_row, t, pos, vel, 0, relative_speed, phase);
		AoA = eval_AoA(u, relative_speed);
		[pitch, yaw] = vect2angleOl(GUI_row.InOl.' * u);

		if active_stage == 1
			Mach_q = min(max(Mach, min(AER.Mach)), max(AER.Mach));
			AoA_q  = min(max(AoA * 180/pi, min(AER.AoA)), max(AER.AoA));
			Cd = interp2(AER.AoA, AER.Mach, AER.Cd, AoA_q, Mach_q);
		else
			Cd = 0;
		end

		if vrel > 0.1
			drag_vec = 0.5 * atmospheric_density * Cd * AER.Sref * vrel^2 * vers(relative_speed);
		else
			drag_vec = zeros(3, 1);
		end
		drag_mag = norm(drag_vec);

		if isignite_row
			mass_flow_rate = MOT(active_stage).mass_flow_rate * MOT(active_stage).number_of_ignite_engine;
			vacuum_thrust  = MOT(active_stage).vacuum_thrust * MOT(active_stage).number_of_ignite_engine;
			thrust = vacuum_thrust - ambient_pressure * MOT(active_stage).nozzle_exit_area * MOT(active_stage).number_of_ignite_engine;
		else
			mass_flow_rate = 0;
			thrust = 0;
		end

		vectorized_thrust = thrust * u;
		not_gravitational_acceleration = vectorized_thrust / mass - drag_vec / mass;
		acc_mag      = norm(not_gravitational_acceleration);
		acc_prop_mag = norm(vectorized_thrust / mass);

		pdyn = 0.5 * atmospheric_density * vrel^2;
		flux = 0.5 * atmospheric_density * vrel^3;   % flusso termico free-molecular semplificato (~1/2 rho v^3)

		fpa     = eval_fpa(pos, vel);
		fpa_rel = eval_fpa(pos, relative_speed);

		theMdot(k)     = mass_flow_rate;
		theAltitude(k) = lla(3);
		theVREL_ECI(k, :) = relative_speed.';
		thePitch(k)    = pitch;
		theYaw(k)      = yaw;
		theAOA(k)      = AoA;
		theMach(k)     = Mach;
		thePdyn(k)     = pdyn;
		theFlux(k)     = flux;
		theThrust(k)   = thrust;
		theDrag(k)     = drag_mag;
		theAcc(k)      = acc_mag;
		theAccProp(k)  = acc_prop_mag;
		theGuidFlag(k) = phase;
		theVrel(k)     = vrel;
		theFPARel(k)   = fpa_rel;
		theFPA(k)      = fpa;
		theNGV(k)      = acc_mag;
		stage(k)       = row_stage;
		Flag_HSSep(k)  = (phase >= 5);   % fairing rilasciata con lo stadio 1 (fine fase 4)
		Flag_MPLSep(k) = false;          % deployment payload non simulato (fuori scope)

		% Longitudine/latitudine "vere": de-rotazione del frame In (frozen
		% a t0) rispetto alla Terra reale, ruotata di omega_E*t da t0
		% (cart2geo/lla(2) da' invece la longitudine "inerziale", CLAUDE.md
		% §6 / nota in cart2geo.m).
		lon_true = wrapToPi(lla(2) - ENV.omega_E * t);
		theLON(k) = lon_true;
		theLAT(k) = lla(1);

		if k == 1
			theDV_drag(k) = 0;
			theDV_Prop(k) = 0;
		else
			dt = T(k) - T(k-1);
			dv_drag_acc = dv_drag_acc + 0.5 * (theDrag(k) / mass + theDrag(k-1) / theMass(k-1)) * dt;
			dv_prop_acc = dv_prop_acc + 0.5 * (acc_prop_mag + theAccProp(k-1)) * dt;
			theDV_drag(k) = dv_drag_acc;
			theDV_Prop(k) = dv_prop_acc;
		end
	end

	theDWR = theAltitude .* 0;   % downrange: distanza al suolo dal sito di lancio
	lat0 = ENV.lat;
	lon0 = ENV.lon;
	for k = 1:N
		central_angle = acos(max(-1, min(1, ...
		                sin(lat0) * sin(theLAT(k)) + cos(lat0) * cos(theLAT(k)) * cos(theLON(k) - lon0))));
		theDWR(k) = central_angle * ENV.Req;
	end

	RES.theTimes    = theTimes;
	RES.theMdot     = theMdot;
	RES.theDWR      = theDWR;
	RES.theX        = theX;
	RES.theY        = theY;
	RES.theZ        = theZ;
	RES.theAltitude = theAltitude;
	RES.theVx       = theVx;
	RES.theVy       = theVy;
	RES.theVz       = theVz;
	RES.thePOS_ECI  = thePOS_ECI;
	RES.theVEL_ECI  = theVEL_ECI;
	RES.theVREL_ECI = theVREL_ECI;
	RES.thePitch    = thePitch;
	RES.theYaw      = theYaw;
	RES.theAOA      = theAOA;
	RES.theMach     = theMach;
	RES.thePdyn     = thePdyn;
	RES.theFlux     = theFlux;
	RES.theThrust   = theThrust;
	RES.theDrag     = theDrag;
	RES.theAcc      = theAcc;
	RES.theAccProp  = theAccProp;
	RES.theGuidFlag = theGuidFlag;
	RES.theVrel     = theVrel;
	RES.theFPARel   = theFPARel;
	RES.theFPA      = theFPA;
	RES.theLON      = theLON;
	RES.theLAT      = theLAT;
	RES.theNGV      = theNGV;
	RES.theDV_drag  = theDV_drag;
	RES.theDV_Prop  = theDV_Prop;
	RES.theMass     = theMass;
	RES.Flag_HSSep  = Flag_HSSep;
	RES.Flag_MPLSep = Flag_MPLSep;
	RES.stage       = stage;
	% Funzione di costo: appartiene al livello dell'ottimizzatore DE
	% esterno (GUIDANCE_VARS.csv), fuori scope per questo simulatore
	% (decisione utente): non calcolata.
	RES.funzione_costo = NaN;
end


function a = wrapToPi(a)
	% wrapToPi  Riporta un angolo (rad) nell'intervallo (-pi, pi].
	a = mod(a + pi, 2*pi) - pi;
end
