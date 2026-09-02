# validation_test — mirror di `reference_LV` (CLAUDE.md §10)

Caso di test **secondario**, ora **identico ai CSV di `reference_LV`** (Falcon
9-like, Cape Canaveral SLC-40). Non è più un lanciatore sintetico separato:
vedi `input/reference_LV/readme.md` per la descrizione di ogni file/valore.

> ⚠️ Motivo del cambio (decisione utente): la versione precedente di questo
> dataset era **sintetica ed equatoriale** (`ENV.lat=ENV.lon=0`, `AZ=90°`,
> `MIS.target_orbital_inclination=0`). Un lancio equatoriale verso EST giace
> *per definizione* nel piano equatoriale (inclinazione naturale ≈ 0), già
> uguale al target: il controllore di piano (`GUI.plane_controller` →
> `PID_actuation.m`, usato solo in `guidance.m` case 6) non veniva **mai
> esercitato**, perché il suo segnale di errore (`target -
> actual_orbital_inclination`) restava sempre nullo. `RES.theInclination`
> risultava quindi costantemente 0.00° — non per un bug, ma perché quel
> dataset non poteva mai far emergere un eventuale controllore rotto.
> `reference_LV` (lat=28.562°N, target=28.4999°) è invece un caso
> **non-degenere**: l'inclinazione naturale del lancio verso EST da quella
> latitudine differisce dal target, quindi in fase 6 il PID ha un errore
> reale da correggere ed è verificabile che il codice reagisca.

## File
Copia verbatim (stesso contenuto, stesso formato) di `ENV.csv`, `GUID.csv`,
`GUIDANCE_VARS.csv`, `LV.csv`, `MIS.csv`, `aero_ascent.csv`, `atmosphere.csv`
da `reference_LV`. Per la descrizione campo per campo vedi
`input/reference_LV/readme.md`.

## Esito atteso (ripetibile)
Con i CSV di questa cartella, lanciando:
```
octave --no-gui --eval "addpath('source'); config.input_dir='input/validation_test'; config.silent=true; RES=simulator(config); write_output_csv(RES,'output');"
```
si ottiene, verificato:
- Attraversamento completo delle fasi 1→8 (nessun blocco), trigger corretti (nessun evento spurio).
  Le fasi 7 (Keplerian transfer) e 8 (Injection in target orbit) sono state aggiunte
  successivamente al primo run di questo dataset (CLAUDE.md §5): sono **istantanee**
  (una sola riga ciascuna in `RES`, non integrate via `ode45`), quindi la missione ora
  prosegue oltre l'apogeo target invece di fermarsi lì.
- Staging: `active_stage` 1→2 a fine fase 4 (t≈142 s), `RES.stage` coerente con la tabella unica CLAUDE.md §5.
- Nessun `NaN`/numero immaginario in nessun campo di `RES`.
- Massa monotona non crescente, con **due** salti attesi: separazione 1° stadio+fairing
  (fine fase 4) e consumo propellente del burn impulsivo di injection (fase 8); quota sempre ≥0.
- **Chiusura di missione per injection completata** (`END_INSERTION`, non crash, non
  esaurimento propellente): fine simulazione a t≈442.8 s, quota≈404.8 km, massa
  finale≈24.1 t (ben sopra il floor `Minert2+Mpayload`=8 t: propellente stadio 2 non
  esaurito, margine residuo).
  Fase 6 (`Boost 1`) porta l'apogeo osculante a 400.000 km (target) lasciando il
  perigeo ancora molto basso/suborbitale (orbita di trasferimento eccentrica); fase 7
  (`LCP`) propaga kepleriano fino all'apogeo reale di quell'orbita; fase 8 (`Boost 2`)
  esegue il burn impulsivo di circolarizzazione/correzione piano — pattern standard a
  due burn (Hohmann-like). Risultato **entro tolleranza sui tre target** (1 km in
  quota, 0.1° in inclinazione): `RES.theApogeeAltitude(end)`=400.000 km,
  `RES.thePerigeeAltitude(end)`=200.000 km = `MIS.perigee_altitude_target`,
  `RES.theInclination(end)`=28.5000° ≈ `MIS.target_orbital_inclination` (28.4999°).
- `RES.theInclination`: parte da ≈28.40° (non 28.562°: è la latitudine
  **geocentrica**, non geodetica, coerente con `geo2cart`/`eval_inclination.m`
  su ellissoide WGS84 — vedi nota sotto) e resta pressoché costante nelle
  fasi 1-5 (il thrust è sempre nel piano di lancio, `yaw` bloccato a
  `GUI.launch_azimuth`: nessuna manovra fuori piano prevista prima della fase
  6). In fase 6 il PID di piano la muove di un'entità piccola (~28.41° a fine
  fase 6, non ancora a target): **non era trattato come bug** nella prima versione
  di questo test (fasi 1-6 soltanto, mancava la correzione finale). Con le fasi
  7-8 aggiunte, il burn impulsivo di fase 8 (`injection_target_orbit.m`) seleziona
  esplicitamente il piano orbitale compatibile con `MIS.target_orbital_inclination`,
  e l'inclinazione finale converge al target (28.50°): la correzione residua che
  mancava al PID di fase 6 viene chiusa dalla fase 8, per costruzione del design a
  due burn.

### Nota: latitudine geocentrica vs geodetica
`eval_inclination.m` calcola `acos(h_z/|h|)` da `pos`/`vel` cartesiani
(geometria pura, quindi intrinsecamente "geocentrica"). `ENV.lat` in
`ENV.csv`/`geo2cart.m` è invece la latitudine **geodetica** (convenzione
WGS84 standard). Sull'ellissoide WGS84 (schiacciamento `f`), la relazione è
`tan(lat_geocentrica) = (1-f)² · tan(lat_geodetica)`: a 28.562° geodetici
corrispondono 28.40° geocentrici — esattamente il valore osservato a inizio
missione (corotazione pura, nessuna componente di velocità fuori dal piano
di corotazione). Nessuna azione richiesta: è la geometria attesa per un
ellissoide oblato, non una discrepanza fra `ENV.lat` letto e usato.
