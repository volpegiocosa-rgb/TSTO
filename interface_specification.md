# Interface Specification

Specifica di interfaccia del simulatore (veicolo di lancio a **due stadi**, propulsione a liquido).

> **Convenzione (importante).** Il codice di calcolo (`eom.m`, `guidance.m`) usa **solo** i nomi
> interni raccolti nel contenitore `other` (`ENV / AER / MOT / GUI / MIS`). Gli **input** sono forniti
> come **file CSV** (delimitati da spazio) nella cartella del dataset (es. `input/reference_LV`) e
> letti da `interface.m`, che costruisce `other`. Gli **output** sono raccolti nella struct `RES`.
> Le vecchie struct del documento sorgente (`LV / GUID / TARG / CONSTR`) **non sono più usate** come
> interfaccia: restano solo come riferimento storico.

---

## 1. Firme delle function

```
interface(input_dir)                                             -> other
eom(t, y, other)                                                 -> dy
simulator(config)                                                -> RES
guidance(MIS, ENV, GUI, t, pos, vel, AoA, relative_speed, phase) -> uIn
create_output(T, Y, other)                                       -> RES
plotter(T, Y, RES)                                               -> <!-- PLACEHOLDER: nessun output / handle figure? -->
```

- `config.input_dir` → path alla cartella dei CSV di input (es. `input/reference_LV`).
- `config.tmax_phase`, `config.silent` → opzioni di run.

---

## 2. INPUT — file CSV (convenzione codice)

Tutti i file sono **CSV delimitati da spazio**, prima riga = header.
`interface.m` li legge e popola `other`. Mapping completo in §4.

### 2.1 `LV.csv` — veicolo (scalari: `name value unit note`) → `AER`, `MOT`
Valori **per singolo motore** (il codice moltiplica per `n_engine`).

| Campo | Unità | → `other` | Note |
|-------|-------|-----------|------|
| `Sref` | m² | `AER.Sref` | area di riferimento aerodinamica |
| `Mfairing` | kg | (uso massa) | massa fairing |
| `M0` | kg | (uso massa) | massa al liftoff, **esclusa P/L** |
| `Minert1` / `Minert2` | kg | (uso massa) | massa a secco per stadio |
| `MProp1` / `MProp2` | kg | (uso massa) | propellente per stadio |
| `Thrust1` / `Thrust2` | N | `MOT(k).vacuum_thrust` | spinta vuoto per motore |
| `MR1` / `MR2` | kg/s | `MOT(k).mass_flow_rate` | portata per motore |
| `Aexit1` / `Aexit2` | m² | `MOT(k).nozzle_exit_area` | area uscita ugello |
| `n_engine1` / `n_engine2` | — | `MOT(k).number_of_ignite_engine` | numero motori per stadio |

### 2.2 `ENV.csv` — ambiente (scalari) → `ENV`
| Campo | Unità | → `other` | Note |
|-------|-------|-----------|------|
| `Req` | m | `ENV.Req` | WGS84 raggio equatoriale |
| `Rpole` | m | `ENV.Rpole` | WGS84 raggio polare |
| `f` | — | `ENV.f` (+ `ENV.wgs84=[Req,f]`) | WGS84 schiacciamento |
| `omega_E` | rad/s | `ENV.omega_E` | rotazione terrestre |
| `mu` | m³/s² | `ENV.mu` | parametro gravitazionale GM |
| `lat` / `lon` / `hpad` | rad / rad / m | `ENV.lat/lon/hpad` | sito di lancio |

### 2.3 `atmosphere.csv` — atmosfera vs quota (tabellare) → `ENV`
Colonne: `altitude T PAtm rho Vsound` (US Standard Atmosphere 1976).
→ `ENV.altitude`, `ENV.ambient_pressure` (`PAtm`), `ENV.atmospheric_density` (`rho`),
`ENV.sound_speed` (`Vsound`). Interpolate in `eom.m` via `interp1`.

### 2.4 `aero_ascent.csv` — aerodinamica di salita (griglia 2D) → `AER`
Layout: **righe = Mach**, **colonne = AoA [deg]** (header `Mach\AoA_deg  a1 a2 ...`).
→ `AER.Mach` (righe), `AER.AoA` (colonne), `AER.Cd` (matrice) per `interp2` in `eom.m`.

### 2.5 `GUID.csv` — guida non ottimizzata (scalari) → `GUI`
`tburnout*` **rimossi** (nel liquido il burnout = esaurimento propellente = trigger fase 4).

| Campo | Unità | → `other` | Note |
|-------|-------|-----------|------|
| `AZ` | rad | `GUI.launch_azimuth` | azimut di lancio |
| `timeHS_Sep_control` | s | `GUI.timeHS_Sep_control` | controllo separazione fairing |
| `flux_HS_Sep` | W/m² | `GUI.flux_HS_Sep` | soglia flusso separazione fairing |

### 2.6 `GUIDANCE_VARS.csv` — variabili di guida (scalari) → `GUI`
Variabili normalmente ottimizzate dalla Differential Evolution; qui fissate per un test
deterministico. Coprono tutti i campi `GUI.*` usati nei `case` di `guidance.m`.

| Campo | → `other` |
|-------|-----------|
| `zkick` | `GUI.zkick` |
| `pitch_over_starting` | `GUI.pitch_over_starting` |
| `pitch_c1`, `pitch_c2` | `GUI.pitch = [c1, c2]` |
| `transition_starting` | `GUI.transition_starting` |
| `pitch_rate_transition` | `GUI.pitch_rate_transition` |
| `pitch_at_transition` | `GUI.pitch_at_transition` |
| `insertion_starting` | `GUI.insertion_starting` |
| `AoA_rate` | `GUI.AoA_rate` |
| `plane_controller_kp/kd/ki` | `GUI.plane_controller = [kp, kd, ki]` |

### 2.7 `MIS.csv` — missione (scalari) → `MIS`
| Campo | Unità | → `other` | Note |
|-------|-------|-----------|------|
| `apogee_altitude_target` | m | `MIS.apogee_altitude_target` | trigger fase 6 |
| `perigee_altitude_target` | m | `MIS.perigee_altitude_target` | perigeo target |
| `target_orbital_inclination` | rad | `MIS.target_orbital_inclination` | inclinazione target |

---

## 3. OUTPUT — struct `RES`

La maggior parte degli output è contenuta nella struct `RES`, costruita da `create_output.m`.

### 3.1 `RES` — Risultati

| Variabile | Descrizione | Unità | Note |
|-----------|-------------|-------|------|
| `RES.theTimes` | Tempo | s | |
| `RES.theMdot` | Portata massica | kg/s | |
| `RES.theDWR` | Downrange | m | |
| `RES.theX` / `theY` / `theZ` | Posizione in ISG | m | |
| `RES.theAltitude` | Quota | m | R − Req |
| `RES.theVx` / `theVy` / `theVz` | Velocità in ISG | m/s | |
| `RES.thePOS_ECI` | Vettore posizione in ECI | m | |
| `RES.theVEL_ECI` | Vettore velocità in ECI | m/s | |
| `RES.theVREL_ECI` | Vettore velocità relativa in ECI | m/s | |
| `RES.thePitch` | Pitch in ISG | rad | |
| `RES.theYaw` | Yaw in ISG | rad | |
| `RES.theAOA` | Angolo d'attacco | rad | |
| `RES.theMach` | Mach | N/A | |
| `RES.thePdyn` | Pressione dinamica | Pa | |
| `RES.theFlux` | Flusso termico | W/m² | |
| `RES.theThrust` | Spinta | N | |
| `RES.theDrag` | Forza di resistenza | N | |
| `RES.theAcc` | Accelerazione non gravitazionale | m/s² | |
| `RES.theAccProp` | Accelerazione propulsiva | m/s² | |
| `RES.theGuidFlag` | Flag di guida (allineato ai `case` di `guidance.m`) | N/A | 1=vertical rise, 2=pitch over, 3=transizione al gravity turn, 4=gravity turn, 5=coasting, 6=insertion. Vedi §3.3 |
| `RES.theVrel` | Velocità relativa | m/s | |
| `RES.theFPARel` | Flight path angle relativo | rad | |
| `RES.theFPA` | Flight path angle assoluto | rad | |
| `RES.theLON` | Longitudine | rad | |
| `RES.theLAT` | Latitudine | rad | |
| `RES.theNGV` | Velocità non gravitazionale | m/s | |
| `RES.theDV_drag` | Delta-v da drag | m/s | |
| `RES.theDV_Prop` | Delta-v propulsivo | m/s | |
| `RES.theIncidence` | Incidence: componente in piano di pitch dell'AoA totale, rispetto all'assetto comandato corrente | rad | richiesto da `plot_list.md` |
| `RES.theSideslip` | Sideslip: componente in piano di yaw dell'AoA totale, rispetto all'assetto comandato corrente | rad | richiesto da `plot_list.md` |
| `RES.theApogeeAltitude` | Quota di apogeo dell'orbita osculante corrente | m | richiesto da `plot_list.md` |
| `RES.theInclination` | Inclinazione orbitale osculante corrente | rad | richiesto da `plot_list.md` |
| `RES.theMass` | Massa | kg | |
| `RES.Flag_HSSep` | Flag separazione fairing | N/A | |
| `RES.Flag_MPLSep` | Flag separazione payload | N/A | |
| `RES.stage` | Identifica lo stadio | N/A | 1=stadio 1, 10=coasting 1, 2=stadio 2, 20=coasting 2 |
| `RES.funzione_costo` | Ultimo valore della funzione di costo | N/A | solo l'ultimo valore |

### 3.2 Vettore di stato `y` (integrato, sorgente degli output)
| Componente | Nome | Unità | Significato |
|------------|------|-------|-------------|
| `y(1:3)` | `pos` | m | Posizione (ECEF/ECI) |
| `y(4:6)` | `vel` | m/s | Velocità |
| `y(7)` | `mass` | kg | Massa istantanea |
| `y(8)` | `dv` | m/s | Delta-v accumulato |

### 3.3 Tabella unica: fase ↔ `theGuidFlag` ↔ `case guidance.m` ↔ `active_stage`
Sorgente di verità: il **codice** (`guidance.m`). La fase 0 (Lift-off) **non è simulata**
(calcolata in `simulator.m`, vedi CLAUDE.md §5).

| Fase | Nome | `guidance.m` case | `RES.theGuidFlag` | `active_stage` | `RES.stage` |
|------|------|-------------------|-------------------|----------------|-------------|
| 0 | Lift-off | — (non simulata) | — | 1 | 1 |
| 1 | Vertical-rise | `case 1` | 1 | 1 | 1 |
| 2 | Pitch over | `case 2` | 2 | 1 | 1 |
| 3 | Transizione al gravity turn | `case 3` | 3 | 1 | 1 |
| 4 | Gravity turn | `case {4,5}` | 4 | 1 (brucia `MOT(1)` fino a esaurimento = trigger) | 1 |
| 5 | Coasting | `case {4,5}` | 5 | **2** (switch a fine fase 4) | 20 |
| 6 | Insertion in transfer orbit | `case 6` | 6 | 2 | 2 |

---

## 4. Mapping `interface.m` (CSV → `other`) — riepilogo

| File CSV | Campo | → `other` |
|----------|-------|-----------|
| `ENV.csv` | `Req/Rpole/f/omega_E/mu/lat/lon/hpad` | `ENV.*` (+ `ENV.wgs84=[Req,f]`) |
| `atmosphere.csv` | `altitude/PAtm/rho/Vsound` | `ENV.altitude/ambient_pressure/atmospheric_density/sound_speed` |
| `LV.csv` | `Sref` | `AER.Sref` |
| `aero_ascent.csv` | griglia | `AER.Mach/AoA/Cd` |
| `LV.csv` | `Thrust1/MR1/Aexit1/n_engine1` | `MOT(1).vacuum_thrust/mass_flow_rate/nozzle_exit_area/number_of_ignite_engine` |
| `LV.csv` | `Thrust2/MR2/Aexit2/n_engine2` | `MOT(2).*` |
| `GUID.csv` | `AZ/timeHS_Sep_control/flux_HS_Sep` | `GUI.launch_azimuth/timeHS_Sep_control/flux_HS_Sep` |
| `GUIDANCE_VARS.csv` | `zkick/pitch_*/*_starting/rate/plane_controller_*` | `GUI.*` (`pitch` e `plane_controller` come vettori) |
| `MIS.csv` | `apogee/perigee/inclination` | `MIS.*` |

### Runtime (impostati da `simulator.m`, non da CSV)
| Campo | Significato |
|-------|-------------|
| `other.phase` | Fase di volo corrente (1..6) |
| `other.isignite` | Motore acceso (`true`) tranne fase 5 |
| `other.GUI.active_stage` | Stadio attivo (1 → 2 a fine fase 4) |
| `other.GUI.last_pitch` / `last_yaw` | Memoria assetto |

---

## 5. TODO / punti aperti
- `AER.Cd` come griglia 2D `Cd(Mach, AoA)`: `interface.m` la carica da `aero_ascent.csv`;
  verificare che `interp2` in `eom.m` usi l'ordine `(Mach, AoA)` coerente.
- `GUI.InOl` (rotazione `Ol → In`): da costruire in `interface.m` da `lat/lon/AZ` (placeholder).
- `RES.stage` (1/10/2/20) vs `active_stage` (1/2): mantenere coerenti nel fasatore.
- Distinzione fase 4 (gravity turn) vs fase 5 (coasting): stesso `case {4,5}` in `guidance.m`,
  ma `theGuidFlag` deve valere 4 o 5 in base a `phase`, non al `case`.
- Formato file di output CSV (ordine colonne, header) per `/output`: da definire.
