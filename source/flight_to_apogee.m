function [tf,yf] = flight_to_apogee(t0,y0,ENV)
%FLIGHT_TO_APOGEE Propaga lo stato corrente fino all'apogeo.
%
% Metodo:
%   - identificazione dell'apogeo tramite elementi orbitali
%   - calcolo tempo residuo mediante equazione di Keplero
%   - propagazione finale mediante variabili universali
%
% INPUT
%   t0      [s]
%   y0      [6x1] = [r;v]
%   ENV.mu  [m^3/s^2]
%   ENV.Req [m] (non utilizzato ma mantenuto per compatibilità)
%
% OUTPUT
%   tf      [s] tempo assoluto all'apogeo
%   yf      [6x1] stato inerziale all'apogeo
%
% NOTE
%   Valido per orbite ellittiche (e < 1).

mu = ENV.mu;

r0 = y0(1:3);
v0 = y0(4:6);

r = norm(r0);
v = norm(v0);

%% ------------------------------------------------------------------------
% Elementi orbitali
%% ------------------------------------------------------------------------

hvec = cross(r0,v0);
h    = norm(hvec);

evec = ((v^2 - mu/r).*r0 - dot(r0,v0).*v0)/mu;
e    = norm(evec);

if e >= 1
    error('flight_to_apogee:NonElliptic',...
        'L''orbita non è ellittica (e >= 1).');
end

energy = v^2/2 - mu/r;

a = -mu/(2*energy);

%% ------------------------------------------------------------------------
% True anomaly attuale
%% ------------------------------------------------------------------------

cosNu0 = dot(evec,r0)/(e*r);
cosNu0 = max(-1,min(1,cosNu0));

nu0 = acos(cosNu0);

if dot(r0,v0) < 0
    nu0 = 2*pi - nu0;
end

%% ------------------------------------------------------------------------
% Anomalia eccentrica attuale
%% ------------------------------------------------------------------------

E0 = 2*atan2( ...
    sqrt(1-e)*sin(nu0/2), ...
    sqrt(1+e)*cos(nu0/2));

E0 = mod(E0,2*pi);

%% ------------------------------------------------------------------------
% Anomalia eccentrica all'apogeo
%% ------------------------------------------------------------------------

Eapo = pi;

%% ------------------------------------------------------------------------
% Tempo da stato attuale ad apogeo
%% ------------------------------------------------------------------------

M0   = E0   - e*sin(E0);
Mapo = Eapo - e*sin(Eapo);

n = sqrt(mu/a^3);

dM = Mapo - M0;

if dM < 0
    dM = dM + 2*pi;
end

dt_apo = dM/n;

%% ------------------------------------------------------------------------
% Propagazione finale mediante variabili universali
%% ------------------------------------------------------------------------

rf_vf = propagate_universal(r0,v0,dt_apo,mu);

tf = t0 + dt_apo;

yf = rf_vf;

end


%% =========================================================================
function yf = propagate_universal(r0,v0,dt,mu)
% Propagazione Kepleriana tramite variabili universali.

r0n = norm(r0);
v0n = norm(v0);

vr0 = dot(r0,v0)/r0n;

alpha = 2/r0n - v0n^2/mu;

%% ------------------------------------------------------------------------
% Guess iniziale
%% ------------------------------------------------------------------------

if abs(alpha) > 1e-12
    chi = sqrt(mu)*abs(alpha)*dt;
else
    chi = sqrt(mu)*dt/r0n;
end

%% ------------------------------------------------------------------------
% Newton-Raphson su chi
%% ------------------------------------------------------------------------

tol = 1e-10;
maxIter = 100;

for k = 1:maxIter

    z = alpha*chi^2;

    C = stumpffC(z);
    S = stumpffS(z);

    F = ...
        r0n*vr0/sqrt(mu)*chi^2*C + ...
        (1-alpha*r0n)*chi^3*S + ...
        r0n*chi - ...
        sqrt(mu)*dt;

    dF = ...
        r0n*vr0/sqrt(mu)*chi*(1-z*S) + ...
        (1-alpha*r0n)*chi^2*C + ...
        r0n*(1-z*C);

    delta = F/dF;

    chi = chi - delta;

    if abs(delta) < tol
        break
    end

end

%% ------------------------------------------------------------------------
% Lagrange coefficients
%% ------------------------------------------------------------------------

z = alpha*chi^2;

C = stumpffC(z);
S = stumpffS(z);

f = 1 - chi^2/r0n*C;

g = dt - chi^3*S/sqrt(mu);

rf = f*r0 + g*v0;

rn = norm(rf);

fdot = sqrt(mu)/(rn*r0n)*(alpha*chi^3*S - chi);

gdot = 1 - chi^2/rn*C;

vf = fdot*r0 + gdot*v0;

yf = [rf;vf];

end


%% =========================================================================
function C = stumpffC(z)

if z > 1e-12
    s = sqrt(z);
    C = (1-cos(s))/z;

elseif z < -1e-12
    s = sqrt(-z);
    C = (cosh(s)-1)/(-z);

else
    C = 1/2;
end

end


%% =========================================================================
function S = stumpffS(z)

if z > 1e-12
    s = sqrt(z);
    S = (s-sin(s))/s^3;

elseif z < -1e-12
    s = sqrt(-z);
    S = (sinh(s)-s)/s^3;

else
    S = 1/6;
end

end