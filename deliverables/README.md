# WETI 2026 — Etap 2: paczka dla Komisji

Termin: **19.06.2026, 15:00**. Poniżej checklista wymaganych plików i co już jest gotowe.

## Wymagane elementy (z regulaminu Etapu 2)

| # | Element | Status | Lokalizacja |
|---|---------|--------|-------------|
| 1 | Prototyp urządzenia (fizyczny) | ✅ zbudowany, działa | — (sprzęt) |
| 2 | Pliki produkcyjne PCB — **Gerber** | ✅ dodane (zip, v17) | `gerber/` |
| 3 | Pliki źródłowe CAD — schemat + PCB | ✅ dodane (.sch + .brd) | `cad/` |
| 4 | Oprogramowanie — źródło + plik końcowy | ✅ gotowe | `firmware/` |
| 5 | Dokumentacja | ✅ gotowa | `../dokumentacja-etap2/` |

Gerber (zip) zawiera komplet warstw: copper top/bottom, soldermask, silkscreen,
solderpaste, profil (obrys), drill (.xln) oraz pliki PnP/assembly — gotowe do
produkcji. CAD: `bezkontaktowy_miernik_odległości.sch` + `...brd` (EAGLE).

## 2 + 3: Referencja eksportu (zrobione)

Poniższa instrukcja pozostaje jako referencja, gdyby trzeba było wygenerować
Gerbery ponownie po zmianie PCB.

### Skopiuj źródła CAD → `cad/`
- `WETI.sch` (schemat)
- `WETI.brd` (płytka)

### Wyeksportuj Gerber → `gerber/` (EAGLE: File → CAM Processor)
Użyj wbudowanego job `gerber_2-layer` (albo `gerb274x`) i wygeneruj warstwy:

| Plik | Warstwa EAGLE | Opis |
|------|---------------|------|
| `*.GTL` | Top | miedź górna |
| `*.GBL` | Bottom | miedź dolna |
| `*.GTS` | tStop | soldermask górny |
| `*.GBS` | bStop | soldermask dolny |
| `*.GTO` | tPlace + tNames | opis górny (silk) |
| `*.GBO` | bPlace + bNames | opis dolny (silk) |
| `*.GTP` | tCream | pasta górna (opcjonalnie) |
| `*.GKO` / `*.GML` | Dimension | obrys płytki |
| `*.TXT` / `*.XLN` | Drill (Excellon) | otwory |

Po eksporcie sprawdź wynik w darmowym wieweru Gerber (np. **gerbv** albo
**JLCPCB/Aisler online viewer**) — czy warstwy się pokrywają i obrys jest zamknięty.

## 4: Oprogramowanie — gotowe ✅

W katalogu `../firmware/`:
- `main.c` — kod źródłowy (AVR-GCC, rejestrowo, ATtiny1616)
- `main.hex` — plik wynikowy (Intel HEX) do wgrania
- `Makefile` — budowanie i programowanie

Budowanie i wgranie:
```
cd firmware
make
avrdude -c jtag2updi -p attiny1616 -P COM5 -b 115200 -U flash:w:main.hex:i
```
(lub `-c serialupdi` dla programatora SerialUPDI/CP2102)

## 5: Dokumentacja — gotowa ✅

`../dokumentacja-etap2/WETI_Kacper_Popko_Etap2.tex` (+ skompilowany `.pdf`).

## Przed wysłaniem — checklista końcowa
- [ ] `cad/WETI.sch` i `cad/WETI.brd` wgrane
- [ ] `gerber/` zawiera wszystkie warstwy + drill, sprawdzone w viewerze
- [ ] `firmware/main.hex` aktualny (po `make`)
- [ ] `dokumentacja-etap2/*.pdf` skompilowany
- [ ] zdjęcia/rendery prototypu dołączone
