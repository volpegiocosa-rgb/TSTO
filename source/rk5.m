function [t, y, te, ye, ie] = rk5(f, t0, tmin, tmax, tend, y0, other, eventFcn, stepFcn)
% RK5 Fifth-order Runge-Kutta solver (Butcher, passo variabile CINEMATICO)
%   con localizzazione di zero-crossing per gli eventi, stile ode45.
%
%   Motivazione (rif. CLAUDE.md solver_project S11 Fase 5, sessione
%   fix-ode-hang): ode45/ode23s a controllo d'errore locale possono restare
%   bloccati per ore su candidati x dove la dinamica resta quasi-inerziale
%   per una finestra simulata lunga (fase 3 con pitch_rate_transition~0,
%   causa concreta diagnosticata via logging per-eval) -- il controllore
%   d'errore riduce il passo a oltranza per soddisfare RelTol/AbsTol senza
%   che questo comporti alcun vantaggio pratico (i vincoli di missione
%   hanno tolleranze late, 3% del target). rk5.m sostituisce il controllo
%   d'errore con un passo CINEMATICO (stepFcn, tipicamente kinematic_step.m
%   in questo repo: passo ~ |v|/|a|) clippato a [tmin,tmax]: il costo
%   computazionale peggiore per fase e' quindi LIMITATO per costruzione a
%   (tend-t0)/tmin passi, indipendentemente da quanto la fisica lo
%   richiederebbe per un'accuratezza stile ode45.
%
% Inputs:
%   f        - Function handle per l'ODE: dy/dt = f(t, y)  (2 argomenti,
%              stessa convenzione di chiamata gia' in uso con ode45 in
%              simulator.m, es. @(t,y) eom(t,y,other): 'other' e' gia'
%              catturato per closure, non serve ripassarlo qui)
%   t0       - Istante iniziale
%   tmin     - Passo minimo (rete di sicurezza sul costo peggiore: il
%              numero di passi per fase e' <= (tend-t0)/tmin)
%   tmax     - Passo massimo (evita passi troppo grandi nei tratti calmi,
%              a scapito della localizzazione degli eventi)
%   tend     - Istante di fine finestra di sicurezza (se la fase non
%              termina prima per via di un evento, applicare una durata
%              di sicurezza per evitare lo stallo -- stesso ruolo di
%              tspan(2) con ode45, gia' in uso come config.tmax_phase)
%   y0       - Vettore colonna delle condizioni iniziali
%   other    - Struct contenitore passata a eventFcn/stepFcn (NON a f, che
%              la riceve gia' via closure -- vedi sopra)
%   eventFcn - Handle event, stessa convenzione 'Events' di ode45 ma con
%              'other' esplicito:
%              [value, isterminal, direction] = eventFcn(t, y, other)
%   stepFcn  - Handle che propone il passo NON ancora clippato:
%              h = stepFcn(t, y, other)  (il clip a [tmin,tmax] e il
%              troncamento a tend sono responsabilita' di rk5.m, non dello
%              stepFcn, cosi' che diverse strategie di stepFcn restino
%              intercambiabili senza duplicare la logica di sicurezza)
%
% Outputs:
%   t, y     - Istanti e matrice di stato (righe = istanti, colonne = stato)
%   te, ye   - Istante e stato in cui si sono verificati gli eventi
%   ie       - Indice della condizione di evento che ha innescato lo stop

    y0 = y0(:);

    n      = 1000;   % capacita' iniziale preallocata; raddoppiata se insufficiente
    t      = zeros(n, 1);
    y      = zeros(n, length(y0));
    y(1,:) = y0.';
    t(1)   = t0;
    n_step = 1;

    % Rete di sicurezza sul numero di passi: con h sempre >= tmin (clip
    % sotto), il ciclo termina comunque entro questo numero di iterazioni;
    % l'errore qui sotto puo' scattare solo per un bug futuro su h/tmin,
    % non in condizioni normali (analogo a max_iterations in simulator.m).
    max_steps = ceil((tend - t0) / tmin) + 10;

    te = []; ye = []; ie = [];

    hasEvents = (nargin >= 8) && ~isempty(eventFcn);

    if hasEvents
        [val_old, isterminal, direction] = eventFcn(t(1), y(1,:).', other);
        numEvents = length(val_old);
    end

    while t(n_step) < tend
        if n_step > max_steps
            error('rk5:tooManySteps', ...
                  ['Superato il numero massimo di passi (%d): tmin ' ...
                   'troppo piccolo rispetto a (tend-t0), oppure bug su h.'], ...
                  max_steps);
        end

        tn = t(n_step);
        yn = y(n_step,:).';

        % --- Passo cinematico, poi clip di sicurezza -----------------
        h = stepFcn(tn, yn, other);
        if h < tmin
            h = tmin;
        end
        if h > tmax
            h = tmax;
        end
        if tn + h > tend
            h = tend - tn;
        end

        % --- Butcher's RK5 Coefficients & Stages ---
        k1 = h * f(tn, yn);
        k2 = h * f(tn + 1/4*h, yn + 1/4*k1);
        k3 = h * f(tn + 1/4*h, yn + 1/8*k1 + 1/8*k2);
        k4 = h * f(tn + 1/2*h, yn - 1/2*k2 + k3);
        k5 = h * f(tn + 3/4*h, yn + 3/16*k1 + 9/16*k4);
        k6 = h * f(tn + h, yn - 3/7*k1 + 2/7*k2 + 12/7*k3 - 12/7*k4 + 8/7*k5);

        t_next = tn + h;
        y_next = yn + (7*k1 + 32*k3 + 12*k4 + 32*k5 + 7*k6) / 90;

        n_step = n_step + 1;
        if n_step > size(t, 1)
            t = [t; zeros(size(t,1), 1)];
            y = [y; zeros(size(y,1), size(y,2))];
        end
        t(n_step)   = t_next;
        y(n_step,:) = y_next.';

        % --- Event Detection Processing ---
        if hasEvents
            [val_new, ~, ~] = eventFcn(t_next, y_next, other);
            terminated = false;

            for idx = 1:numEvents
                % Check for a sign change (zero-crossing)
                if val_old(idx) * val_new(idx) <= 0 && val_old(idx) ~= val_new(idx)

                    % Verify direction constraint matching MATLAB logic:
                    % direction =  1 -> increasing only
                    % direction = -1 -> decreasing only
                    % direction =  0 -> any direction
                    is_correct_dir = (direction(idx) == 0) || ...
                                     (direction(idx) == 1  && val_new(idx) > val_old(idx)) || ...
                                     (direction(idx) == -1 && val_new(idx) < val_old(idx));

                    if is_correct_dir
                        % Localizzazione lineare dell'evento (stessa
                        % semplificazione gia' presente nella bozza
                        % originale: un passo cinematico puo' essere
                        % grosso, quindi meno preciso di ode45 su questo
                        % punto -- accettabile vista la tolleranza di
                        % missione, 3% del target, cfr. CLAUDE.md Fase 5)
                        theta = val_old(idx) / (val_old(idx) - val_new(idx));
                        t_evt = tn + theta * h;
                        y_evt = yn + theta * (y_next - yn);

                        te = [te; t_evt];
                        ye = [ye; y_evt.'];
                        ie = [ie; idx];

                        % Halt integration if marked as a terminal event
                        if isterminal(idx)
                            t = t(1:n_step); t(end) = t_evt;
                            y = y(1:n_step,:); y(end,:) = y_evt.';
                            terminated = true;
                            break;
                        end
                    end
                end
            end

            if terminated, break; end
            val_old = val_new; % Cache values for the next step
        end
    end
    t = t(1:n_step); y = y(1:n_step,:);
end
