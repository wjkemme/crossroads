# ADR-0003: Data-uitwisseling Config ↔ Simulatie

- **Status**: Accepted
- **Datum**: 2026-02-26

## Context
Configuratie en simulatie communiceren via server-endpoints. Zonder versionering en duidelijke validatieregels kan een UI of tool een onveilige of incompatibele configuratie pushen.

## Beslissing
We hanteren een expliciet uitwisselingscontract:
- Config ophalen/bijwerken via `GET/POST /config/api`.
- Simulatiestatus via `GET /snapshot`.
- Commando's via `GET /command?cmd=...` (te migreren naar `POST`).
- Config update-pad: parse -> schema-check -> domeinvalidatie -> apply/rollback.

## Contractregels
- Iedere payload bevat een impliciete contractversie (huidig: v1 op veldniveau).
- Onbekende verplichte velden of typefouten geven `400`.
- Domeinfouten (onveilige combinaties) geven `400` met foutdetails.
- Simulatie blijft doordraaien met laatste geldige configuratie bij apply-fout.

## Consequenties
- Positief: veiligere runtime updates van kruispuntconfiguraties.
- Positief: basis voor toekomstige externe clients/websocket-sync.
- Negatief: meer validatiecode en testcases nodig.

## Open punten
- Migratiepad van command-GET naar command-POST.
- Expliciete `config_version` en `snapshot_version` velden toevoegen.
- Mogelijke WebSocket-eventstream naast polling.
