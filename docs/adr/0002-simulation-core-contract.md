# ADR-0002: Simulatie als deterministic core

- **Status**: Accepted
- **Datum**: 2026-02-26

## Context
De simulator moet voorspelbaar blijven terwijl UI en configuratie los kunnen evolueren. Zonder duidelijk contract tussen tick-loop en externe input ontstaan timing- en raceproblemen.

## Beslissing
De simulatiecore blijft deterministic en tick-gedreven:
- Input: gevalideerde configuratie + commando's (`start`, `stop`, `step`, `reset`) + tijdstap.
- Output: snapshot (`lanes`, `lights`, `metrics`, `sim_time`) als read-model.
- Geen directe UI-state in de core.

## Consequenties
- Positief: reproduceerbare runs en stabiele tests.
- Positief: meerdere UI-clients mogelijk zonder invloed op simulatiestatus.
- Negatief: extra vertaling nodig van interne state naar snapshot DTO.

## Core regels
- Light-transities volgen toegestaan pad (incl. orange duur).
- Safety-check loopt als guard op controller-uitvoer.
- Safety-regels mogen geen layout/routinggedrag muteren: wegstructuur, lane-connections en toegestane bewegingen komen uitsluitend uit configuratie.
- Verkeersgedrag gebruikt die configuratie als bron van waarheid; safety valideert alleen lichttoestanden en transities.
- Snapshot is immutabel voor clients; wijzigingen alleen via commando's/config update.

## Conflict resolution contract
- De simulator gebruikt een route-gebaseerde conflictmatrix als primaire beslisbron voor lichtsturing.
- Bij activatie van een gekozen route (anchor route) worden alleen conflicterende routes actief naar rood gestuurd.
- Niet-conflicterende routes worden ongemoeid gelaten.
- Voorbeeldreferentie: `N->O` conflicteert met `O->W`, maar niet met `O->N`.

## Scheduling en fairness garanties
- Routekeuze gebeurt op adaptieve prioriteit (druk + wachttijdveroudering).
- Starvation is verboden voor routes met aanhoudende vraag; bounded-wait geldt met `Wmax = 20s`.
- Scheduler mag parallelle routes activeren in hetzelfde venster, mits ze niet conflicteren met anchor route en niet onderling conflicteren.
- Groentijd is primair adaptief voor doorstroming; er is geen vaste harde cap zolang er geen conflicterende wachtrijen zijn.
- Fallback-regel: zodra conflicterende routes wachten, geldt een harde maximale effectieve groentijd van `8s` voor de actieve route.
- Bij wissel naar conflicterende route(s) wordt een clearance-fase afgedwongen: eerst afbouw van bestaande groenfase (`Groen -> Oranje -> Rood`), daarna pas conflicterende groenactivatie.

## Transition discipline
- Verplichte volgorde per licht: `Groen -> Oranje -> Rood -> Groen`.
- `Rood -> Oranje` is ongeldig.
- Minimum-groen lock voorkomt direct terugschakelen na activatie.

## Rolafbakening Scheduler vs SafetyChecker
- Scheduler bepaalt prioriteit, conflictvrije routecombinaties en activatiemomenten.
- SafetyChecker is finale autoriteit op toestand/transitievalidatie.
- SafetyChecker corrigeert/keurt af, maar is niet de primaire routeplanner.

## Alternatieven
- UI-gestuurde simulatielogica (afgewezen: koppelt presentatie en domein te sterk).
- Event-only model zonder ticks (afgewezen: complexer voor huidige scope).
