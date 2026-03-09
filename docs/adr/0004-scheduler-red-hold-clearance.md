# ADR-0004 Scheduler red-hold en crossing-clearance

- Status: Accepted
- Datum: 2026-03-09

## Context
- Conflictroutes moeten eerst veilig afgewikkeld worden voordat een nieuwe anchor- of parallel-route groen mag.
- We willen alleen actief kruisend verkeer tellen (voertuigen die al aan het oversteken zijn), niet de wachtende rij.
- Orange blokkeert: een conflictroute in Orange mag geen groen voor de anchor/parallels vrijgeven.

## Beslissing
1) **Red-hold per conflictroute**: een conflictroute moet minimaal 2s onafgebroken op Red hebben gestaan (`route_red_since`), gemeten na transition discipline.
2) **Crossing-clear buffer**: naast Red-hold moet het kruispunt 2s vrij zijn van kruisende voertuigen (`route_conflicts_cleared_at` + 2s buffer).
3) **Gates voor anchor én parallel**: dezelfde checks (Red-hold + crossing-clear buffer) gelden voor de anchor en voor iedere parallel-kandidaat.
4) **Orange blokkeert**: Orange telt als niet-Red; de gate faalt als een conflictroute Orange is.

## Consequenties
- Groen kan pas nadat conflictroutes zowel Red-hold ≥ 2s als crossing-clear ≥ 2s hebben doorstaan.
- Wachtende voertuigen worden niet meegeteld in clearance; enkel actief kruisend verkeer.
- Transition discipline kan groen→orange→red zetten; `route_red_since` wordt pas na discipline gezet en vormt zo de bron voor de 2s hold.

## Alternatieven overwogen
- Wachtende voertuigen meenemen in clearance (afgewezen: vereiste is crossings-only).
- Geen Red-hold (afgewezen: te weinig veiligheidsmarge).
