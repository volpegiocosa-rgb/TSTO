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
- Attraversamento completo delle fasi 1→6 (nessun blocco), trigger corretti (nessun evento spurio).
- Staging: `active_stage` 1→2 a fine fase 4 (t≈142 s), `RES.stage` coerente con la tabella unica CLAUDE.md §5.
- Nessun `NaN`/numero immaginario in nessun campo di `RES`.
- Massa monotona non crescente (con il salto atteso alla separazione 1° stadio+fairing), quota sempre ≥0.
- **Chiusura di missione per apogeo raggiunto** (non crash, non esaurimento propellente):
  fine simulazione a t≈239 s, quota≈267.5 km, massa≈100.3 t, apogeo osculante finale = 400.000 km = `MIS.apogee_altitude_target`.
- `RES.theInclination`: parte da ≈28.40° (non 28.562°: è la latitudine
  **geocentrica**, non geodetica, coerente con `geo2cart`/`eval_inclination.m`
  su ellissoide WGS84 — vedi nota sotto) e resta pressoché costante nelle
  fasi 1-5 (il thrust è sempre nel piano di lancio, `yaw` bloccato a
  `GUI.launch_azimuth`: nessuna manovra fuori piano prevista prima della fase
  6). In fase 6 il PID di piano (`error = target - actual`, qui ≈ +0.10°)
  la muove nella direzione corretta (crescente verso il target), ma di
  un'entità piccola (~0.0016° su 39 s di burn fase 6, dati GUIDANCE_VARS.csv
  nominali): il target non viene raggiunto entro la finestra di missione.
  **Non trattato come bug**: il verso della correzione è corretto e il
  codice (`eval_inclination.m`, `guidance.m` case 6, `PID_actuation.m`) è
  stato riletto e non presenta errori evidenti; l'entità ridotta può essere
  dovuta a guadagni `plane_controller` non ottimizzati (§10.1: "set nominale,
  non ottimizzato") o al tempo di burn insufficiente. Segnalato all'utente,
  non corretto senza indicazione esplicita (i guadagni sono condivisi con
  altri comportamenti di fase 6, es. l'AoA di inserimento).

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
