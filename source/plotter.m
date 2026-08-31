function plotter(T, Y, RES)
	% plotter  Grafici essenziali della simulazione (quota, velocita',
	%          massa, traiettoria, AoA/Mach/pitch, fase). Chiamata da
	%          simulator.m solo se config.silent e' false/assente.
	%
	% Input : T, Y  (storia temporale grezza)
	%         RES   (struct dei risultati, create_output.m)
	% Output: nessuno (crea figure)

	figure('Name', 'Quota e velocita');
	subplot(2, 1, 1);
	plot(RES.theTimes, RES.theAltitude / 1000);
	xlabel('t [s]'); ylabel('quota [km]'); grid on;
	subplot(2, 1, 2);
	vel_mag = sqrt(sum(Y(:, 4:6).^2, 2));
	plot(RES.theTimes, vel_mag / 1000);
	xlabel('t [s]'); ylabel('|v| [km/s]'); grid on;

	figure('Name', 'Massa e stadio');
	subplot(2, 1, 1);
	plot(RES.theTimes, RES.theMass);
	xlabel('t [s]'); ylabel('massa [kg]'); grid on;
	subplot(2, 1, 2);
	plot(RES.theTimes, RES.theGuidFlag, RES.theTimes, RES.stage);
	xlabel('t [s]'); ylabel('flag'); legend('theGuidFlag', 'stage'); grid on;

	figure('Name', 'Aerodinamica e assetto');
	subplot(3, 1, 1);
	plot(RES.theTimes, RES.theMach);
	xlabel('t [s]'); ylabel('Mach'); grid on;
	subplot(3, 1, 2);
	plot(RES.theTimes, RES.theAOA * 180/pi);
	xlabel('t [s]'); ylabel('AoA [deg]'); grid on;
	subplot(3, 1, 3);
	plot(RES.theTimes, RES.thePitch * 180/pi, RES.theTimes, RES.theYaw * 180/pi);
	xlabel('t [s]'); ylabel('[deg]'); legend('pitch', 'yaw'); grid on;

	figure('Name', 'Traiettoria 3D (frame In)');
	plot3(Y(:, 1) / 1000, Y(:, 2) / 1000, Y(:, 3) / 1000);
	xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
	grid on; axis equal;
end
