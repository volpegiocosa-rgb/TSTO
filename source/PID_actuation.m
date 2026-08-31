function u = PID_actuation(kp, kd, ki, err)
	% PID_actuation  Attuazione del controllore di piano (case 6, insertion).
	%
	% LIMITE NOTO E DOCUMENTATO: 'other' e' passato per valore a ogni
	% chiamata di eom.m dentro ode45 (nessuna variabile globale ammessa,
	% CLAUDE.md §4), quindi non esiste uno stato persistito tra chiamate in
	% cui accumulare un vero integrale dell'errore o stimarne la derivata.
	% Qui i tre guadagni sono applicati tutti all'errore istantaneo (somma
	% pesata), NON un vero PID con memoria. Da estendere in futuro portando
	% l'errore integrato dentro il vettore di stato y (nuova componente),
	% se serve un'azione integrale reale.
	%
	% Input  : kp, kd, ki  (scalari, guadagni, GUI.plane_controller)
	%          err         (scalare, errore di inclinazione, rad)
	% Output : u           (scalare, comando di yaw corretto, rad)

	u = (kp + kd + ki) * err;
end
