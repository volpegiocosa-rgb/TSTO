function h = kinematic_step(t, y, other)
% KINEMATIC_STEP  stepFcn per rk5.m: propone un passo di integrazione a
%   partire da informazioni cinematiche istantanee (velocita', accelerazione),
%   invece di un controllo d'errore locale stile ode45/ode23s.
%
%   IDEA (decisione utente, sessione fix-ode-hang, rif. CLAUDE.md
%   solver_project S11 Fase 5): il passo e' una frazione del tempo
%   caratteristico su cui l'accelerazione corrente cambierebbe in modo
%   apprezzabile la velocita', tau = |v| / |a|.
%   - Dinamica quasi-inerziale (|a| piccola rispetto a |v|): esattamente
%     il regime della fase 3 con pitch_rate_transition~0 che ha causato
%     l'hang di ore (nessuna correzione attiva, il pitch resta costante
%     per una finestra simulata lunga) -- tau e' grande, il passo cresce
%     fino a tmax invece di restare piccolo come con un controllo
%     d'errore a tolleranza fissa.
%   - Dinamica rapida (accensione motore, pitch-over, staging): tau si
%     riduce e il passo si restringe di conseguenza, senza bisogno di un
%     vero controllo d'errore locale.
%   Il clip a [tmin,tmax] e il troncamento a tend NON sono fatti qui:
%   sono responsabilita' di rk5.m (vedi header di quel file).
%
%   ASSUNZIONE DICHIARATA (non validata oltre lo smoke-test nominale +
%   ricontrollo del candidato che ha causato l'hang, sessione
%   fix-ode-hang): frac e' un numero unico globale, non calibrato su un
%   target di accuratezza degli eventi (rif. localizzazione lineare in
%   rk5.m). Se un run a convergenza mostrasse eventi localizzati in modo
%   troppo impreciso, ridurre frac (o altrimenti tmax) e' il primo punto
%   da rivedere -- apertura dichiarata, non chiusa qui.
%
%   INPUT  t, y  : istante e stato correnti (convenzione eom.m: y 8x1,
%                  y(1:3) posizione, y(4:6) velocita', y(7) massa, y(8) dv)
%          other : struct passata a eom.m; other.STEP.frac (opzionale,
%                  default 0.05) e' la frazione di tau usata come passo.
%                  other.eom_fast (opzionale, closure 2 argomenti (t,y)):
%                  se presente viene usata al posto di eom(t,y,other) --
%                  rif. CLAUDE.md solver_project S11 Fase 5, sessione
%                  "requisito 5 minuti": simulator.m la imposta con la
%                  versione compilata (eom_native.oct) quando disponibile,
%                  per non lasciare 1/7 delle chiamate per punto RK5 sul
%                  percorso lento interpretato mentre le altre 6 (k1..k6
%                  di rk5.m) usano gia' la versione veloce.
%   OUTPUT h     : passo proposto, non ancora clippato.

    if isfield(other, 'STEP') && isfield(other.STEP, 'frac')
        frac = other.STEP.frac;
    else
        frac = 0.05;   % TODO: PROVVISORIO -- non calibrato, vedi header
    end

    if isfield(other, 'eom_fast') && ~isempty(other.eom_fast)
        dy = other.eom_fast(t, y);
    else
        dy = eom(t, y, other);
    end
    vel = y(4:6);
    acc = dy(4:6);

    v = norm(vel);
    a = norm(acc);

    if a > 1e-6
        tau = v / a;
    else
        tau = Inf;   % accelerazione trascurabile: nessun vincolo cinematico, decide tmax a valle
    end

    h = frac * tau;
end
