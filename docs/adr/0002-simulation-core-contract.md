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

## Alternatieven
- UI-gestuurde simulatielogica (afgewezen: koppelt presentatie en domein te sterk).
- Event-only model zonder ticks (afgewezen: complexer voor huidige scope).
