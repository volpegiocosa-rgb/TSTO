# validation_test_2 — mirror di `validation_test` con perigeo target 400 km e P/L dimezzato

Caso di test **derivato** da `input/validation_test` (a sua volta mirror di
`reference_LV`, vedi `input/reference_LV/readme.md` per la descrizione campo per
campo). Copia verbatim di `ENV.csv`, `GUID.csv`, `GUIDANCE_VARS.csv`,
`aero_ascent.csv`, `atmosphere.csv` da `validation_test`. Due sole differenze:

| File | Campo | `validation_test` | `validation_test_2` | Motivo |
|------|-------|--------------------|----------------------|--------|
| `MIS.csv` | `perigee_altitude_target` | 200000 m | **400000 m** | orbita target diventa circolare 400×400 km invece di ellittica 200×400 km |
| `LV.csv` | `Mpayload` | 4000 kg | **2000 kg** | 50% del baseline `reference_LV`/`validation_test` (nessun dataset originario definiva una massa payload: baseline scelto e documentato in `input/reference_LV/readme.md`, CLAUDE.md §10.1) |

`apogee_altitude_target` e `target_orbital_inclination` restano invariati (400000 m,
0.49741883 rad).

## Esito osservato (ripetibile)
```
octave --no-gui --eval "addpath('source'); config.input_dir='input/validation_test_2'; config.silent=true; RES=simulator(config); write_output_csv(RES,'output');"
```
- Attraversamento completo delle fasi 1→8, nessun evento spurio, nessun `NaN`/numero
  immaginario in `RES`.
- Massa monotona non crescente, due salti attesi (separazione 1° stadio+fairing a
  fine fase 4; burn impulsivo di injection in fase 8).
- **Chiusura di missione per injection completata** (`END_INSERTION`): fine
  simulazione a t≈434.7 s, quota≈404.8 km, massa finale≈23.4 t (ben sopra il floor
  `Minert2+Mpayload`=6 t: propellente stadio 2 non esaurito).
- Target centrato entro tolleranza (1 km in quota, 0.1° in inclinazione):
  `RES.theApogeeAltitude(end)`=400.000 km, `RES.thePerigeeAltitude(end)`=**400.000 km**
  = `MIS.perigee_altitude_target` (orbita finale circolare, come atteso dal nuovo
  target), `RES.theInclination(end)`=28.5000° ≈ target (28.4999°).
- Rispetto a `validation_test` (perigeo target 200 km): la fase 8 qui deve innalzare
  il perigeo fino a 400 km invece che 200 km (burn più grande in termini di variazione
  di energia orbitale), ma la massa payload dimezzata (2000 kg vs 4000 kg) lascia più
  massa disponibile come propellente utilizzabile a parità di `M0`+`MProp2`: il burn
  resta comunque entro il propellente disponibile (nessuna saturazione,
  `END_INSERTION` non `END_INSERTION_PARTIAL`), terminando leggermente prima
  (t≈434.7 s vs t≈442.8 s di `validation_test`) perché fase 6 raggiunge il trigger di
  apogeo con un burn leggermente più corto (mass ratio diverso per il payload ridotto).

## Nota
Questo dataset non verifica un caso di saturazione del Δv (propellente insufficiente
→ `END_INSERTION_PARTIAL`): con questi valori di `Mpayload`/target il burn di fase 8
ha sempre margine di propellente. Non trattato come limite del test: l'obiettivo
dichiarato (perigeo target 400 km, payload dimezzato) è verificato con successo entro
tolleranza; un caso di saturazione andrebbe costruito con un payload/target più
esigenti, fuori scope di questo dataset.
