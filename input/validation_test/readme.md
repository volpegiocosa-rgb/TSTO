# validation_test — dataset di input sintetico (CLAUDE.md §10)

Caso di test **secondario**, interamente sintetico (nessuna fonte esterna): un
lanciatore a due stadi a liquido molto più piccolo del `reference_LV`, per
verificare il codice su ordini di grandezza diversi (§10.1: "valori
fisicamente plausibili", non necessariamente ottimi).

> ⚠️ Stessa convenzione di `reference_LV`: i CSV usano i nomi interni
> (`ENV / AER / MOT / GUI / MIS`), non le struct del documento di interfaccia.

## File

### `LV.csv` — veicolo sintetico
Valori per **singolo motore**. `M0=100000 kg` = `Minert1+Minert2+MProp1+MProp2+Mfairing`
(4000+1000+70000+24500+500), verificato. T/W al liftoff (SL, 2 motori) ≈ 1.43.
Burn stadio 1 ≈ `MProp1/(n_engine1·MR1)` = 70000/(2·237.9) ≈ 147 s.

### `ENV.csv` — sito equatoriale sintetico
`lat=lon=hpad=0` (caso limite: origine del frame `In` coincide col sito di
lancio a t0). WGS84 standard per il resto.

### `GUID.csv` — `AZ=90°` (lancio verso EST, equatoriale ⇒ inclinazione ≈ 0).

### `GUIDANCE_VARS.csv` — set nominale, non ottimizzato
Come per `reference_LV`: valori plausibili per far girare il test end-to-end,
non una traiettoria ottima. **Non modificare senza ri-verificare**: il sistema
guida (fase 3, transizione al gravity turn) è sensibile ai parametri vicino al
punto in cui il segno di `pitch_rate` (deciso da `eval_aerodynamic_angle` in
`guidance.m` case 3) si inverte — piccole variazioni di `pitch_at_transition`/
`transition_starting` intorno a quella soglia possono cambiare drasticamente
la traiettoria (osservato empiricamente durante la calibrazione: da un
apogeo end-fase-4 di ~300 km a un crash quasi immediato, con un salto netto
tra le due configurazioni). Non è stato necessario ritoccare questi valori
per ottenere un'uscita di missione corretta: vedi `MIS.csv`.

### `MIS.csv` — target di missione
| Campo | Valore | Nota |
|-------|--------|------|
| `apogee_altitude_target` | 400000 m | trigger fase 6 |
| `perigee_altitude_target` | 200000 m | |
| `target_orbital_inclination` | 0.0 rad | lancio equatoriale verso EST |

> Corretto da un refuso iniziale (400/200 invertiti in 200/150): con
> `apogee_altitude_target=200000` l'apogeo osculante superava già i 200 km
> **durante la fase 4** (gravity turn, prima ancora di iniziare la fase 6),
> quindi l'event di fase 6 (che rileva solo un attraversamento dal basso,
> `direction=1`) non poteva mai scattare — la missione proseguiva fino a
> esaurire il margine di manovra e terminava per quota=0 (crash). Con il
> target riallineato a 400 km (stessa convenzione di `reference_LV`), il
> `GUIDANCE_VARS.csv` nominale invariato porta l'apogeo osculante da ~296 km
> (fine fase 4) a 400 km esatti durante la fase 6, senza toccare i parametri
> di guida.

### `atmosphere.csv`, `aero_ascent.csv`
Copiati verbatim da `reference_LV` (tabelle universali, non specifiche del veicolo).

## Esito atteso (ripetibile)
Con i CSV di questa cartella, lanciando:
```
octave --no-gui --eval "addpath('source'); config.input_dir='input/validation_test'; config.silent=true; RES=simulator(config); write_output_csv(RES,'output');"
```
si ottiene, verificato:
- Attraversamento completo delle fasi 1→6 (nessun blocco), trigger corretti (nessun evento spurio).
- Staging: `active_stage` 1→2 a fine fase 4 (t≈147 s), `RES.stage` coerente con la tabella unica CLAUDE.md §5.
- Nessun `NaN`/numero immaginario in nessun campo di `RES`.
- Massa monotona non crescente, quota sempre ≥0.
- **Chiusura di missione per apogeo raggiunto** (non crash, non esaurimento propellente):
  fine simulazione a t≈357 s, quota≈365 km, massa≈20177 kg (ben sopra `Minert2+Mpayload`≈1000 kg),
  apogeo osculante finale = 400.000 km = `MIS.apogee_altitude_target`.
