# Reference LV — dataset di input di riferimento (Falcon 9-like)

Caso di test **primario**. I dati sono **dedotti dalle specifiche SpaceX allegate**
(`SPACEX_FALCON_USERS__GUIDE.pdf`, `SpaceX_Merlin.pdf`): rappresentano un lanciatore
a **due stadi a liquido** LOX/RP-1 (Falcon 9 Full Thrust / Block 5).

> ⚠️ **Convenzione**: i file di input seguono la **convenzione interna del codice**
> (`ENV / AER / MOT / GUI / MIS`), **non** le struct del documento di interfaccia.
> Il documento `interface_specification.md` va usato **solo per gli output** (`RES`).

## Formato
- **CSV delimitati da spazio** (CLAUDE.md §2). Prima riga = header.
- File scalari: colonne `name value unit note`.
- `interface.m` legge questi CSV e popola `other` (ENV/AER/MOT/GUI/MIS).

## File

### `LV.csv` — veicolo → alimenta `MOT` (per stadio) e `AER.Sref`
Valori per **singolo motore** (il codice moltiplica per `n_engine`).
| Campo | Valore | Fonte |
|-------|--------|-------|
| `Sref` | 10.52 m² | area frontale, Ø 3.66 m |
| `Mfairing` | 1900 kg | fairing Falcon 9 |
| `M0` | 550000 kg | massa al liftoff, **esclusa P/L** |
| `Minert1/2` | 25600 / 4000 kg | masse a secco per stadio |
| `MProp1/2` | 411000 / 107500 kg | propellente per stadio |
| `Thrust1` | 981 kN | Merlin 1D, spinta vuoto (per motore) |
| `MR1` | 321.7 kg/s | Merlin 1D, portata (da Isp_vac 311 s) |
| `Aexit1` | 0.665 m² | ugello Merlin 1D (Ø 0.92 m) |
| `n_engine1` | 9 | motori stadio 1 |
| `Thrust2` | 981 kN | MVac, spinta vuoto |
| `MR2` | 287.5 kg/s | MVac, portata (da Isp_vac 348 s) |
| `Aexit2` | 7.07 m² | ugello MVac (Ø ~3.0 m, ε=165) |
| `n_engine2` | 1 | motori stadio 2 |

> Verifica di sanità: burn stadio 1 ≈ `MProp1/(9·MR1)` ≈ **142 s** (coerente col
> MECO ~145 s nella timeline Falcon 9); T/W al liftoff (SL) ≈ **1.41**.

### `ENV.csv` — costanti WGS84 + sito di lancio
| Campo | Valore | Nota |
|-------|--------|------|
| `Req` | 6378137.0 m | WGS84 raggio equatoriale |
| `Rpole` | 6356752.314 m | WGS84 (1−f)·Req |
| `f` | 1/298.257223563 | WGS84 schiacciamento |
| `omega_E` | 7.292115e-5 rad/s | rotazione terrestre |
| `mu` | 3.986004418e14 m³/s² | WGS84 GM (`ENV.mu` in `eom.m`) |
| `lat/lon/hpad` | 28.5620°/−80.5772°/0 | Cape Canaveral SLC-40 (in rad) |

### `GUID.csv` — parametri di guida NON ottimizzati
`tburnout1/2` **rimossi**: nel motore a liquido il burnout è dato
dall'esaurimento del propellente (trigger della fase 4), non da un tempo imposto.
| Campo | Valore | Nota |
|-------|--------|------|
| `AZ` | 1.5708 rad | azimut di lancio (90°, verso EST) |
| `timeHS_Sep_control` | 180 s | tempo min. controllo separazione fairing |
| `flux_HS_Sep` | 1135 W/m² | soglia flusso separazione fairing |

### `GUIDANCE_VARS.csv` — variabili di guida (allineate a `GUI` di `guidance.m`)
Sono le variabili che la Differential Evolution normalmente ottimizza; qui fissate
a un set nominale per un test deterministico. Coprono tutti i campi `GUI.*` usati
nei `case` di `guidance.m`: `zkick`, profilo `pitch(1)/pitch(2)`, istanti di inizio
fase, rate di transizione/AoA, guadagni `plane_controller [kp,kd,ki]`.

### `MIS.csv` — target di missione (ex-TARG)
| Campo | Valore | Nota |
|-------|--------|------|
| `apogee_altitude_target` | 400000 m | LEO 400 km (trigger fase 6) |
| `perigee_altitude_target` | 200000 m | perigeo 200 km |
| `target_orbital_inclination` | 0.49742 rad | 28.5° (lancio da Cape verso EST) |

### `atmosphere.csv` — US Standard Atmosphere 1976
Colonne `altitude T PAtm rho Vsound` (0–100 km). Alimenta i campi
`ENV.ambient_pressure / atmospheric_density / sound_speed` via interpolazione.

### `aero_ascent.csv` — tabella `Cd(Mach, AoA)` 2D
Layout: **righe = Mach**, **colonne = AoA [deg]** (header `Mach\AoA_deg`).
Alimenta `AER.Mach`, `AER.AoA`, `AER.Cd` per l'`interp2` in `eom.m`.
Picco transonico a Mach 1.0–1.2, crescita con l'angolo d'attacco.

## Note aperte
- `number_of_ignite_engine` è fornito qui come `n_engine1/2` in `LV.csv`
  (mancava nella specifica docx): `interface.m` lo mappa su `MOT(k).number_of_ignite_engine`.
- I valori in `GUIDANCE_VARS.csv` sono un **set nominale plausibile**, non ottimizzato:
  servono a far girare il test end-to-end, non a rappresentare una traiettoria ottima.
