# ADR-0001: Configuratie als expliciet domeinmodel

- **Status**: Accepted
- **Datum**: 2026-02-26

## Context
Configuratie van een kruispunt wordt nu gebruikt door meerdere onderdelen: UI, simulator, safety checks en opslag. Zonder expliciet domeinmodel ontstaat drift tussen JSON, database en runtime-objecten.

## Beslissing
We behandelen configuratie als zelfstandig domeinmodel met één canonieke structuur:
- `IntersectionConfig`
- `ApproachConfig`
- `LaneConfig`
- `SignalGroupConfig`

Uitwisseling verloopt via JSON, maar validatie gebeurt op domeinniveau (semantiek, niet alleen syntaxis).

## Consequenties
- Positief: consistente betekenis van lane/signal-velden in hele codebase.
- Positief: eenvoudiger testen van validatie en migraties.
- Negatief: extra mapping-laag nodig tussen DB en domeinobjecten.

## Invarianten
- Elke approach heeft unieke `id`.
- Elke lane heeft unieke `id` binnen de configuratie.
- `to_lane_count >= 1`.
- `allowed_movements` is niet leeg voor verbonden lanes.
- `has_traffic_light` kan alleen waar zijn als `connected_to_intersection` waar is.

## Alternatieven
- Ad-hoc JSON direct in simulator (afgewezen: te foutgevoelig).
- DB-schema als enige waarheid (afgewezen: lekt persistence-details in domeinlogica).
