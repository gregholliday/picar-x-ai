# Obsolete Scripts

These scripts are superseded by the new navigator architecture and can be removed
from the repository. Remove them from your KB before re-ingesting.

## Pi scripts (on Pi, in pi/)

| Script | Replaced By | Reason |
|--------|-------------|--------|
| `picar_agent_v7.py` | `picar_agent.py` | Versioned filename, use canonical name |
| `picar_agent_v8.py` | `picar_agent.py` | Versioned filename, use canonical name |

## Navigator scripts (on Fedora, in navigator/)

| Script | Replaced By | Reason |
|--------|-------------|--------|
| `track_navigator.py` | `roam_navigator.py` + `mission_navigator.py` | Track-based approach abandoned for LiDAR geofencing |
| `track_monitor.py` | `roam_navigator.py` | Simplified monitor replaced by full navigator |
| `picar_navigator.py` | `roam_navigator.py` | Original vision-based navigator, superseded |
| `picar_agent_v7.py` (if in navigator/) | — | Wrong location |

## KB/pipeline scripts (in scripts/)

| Script | Status | Reason |
|--------|--------|--------|
| `ingest_articles.py` | Keep but update | Uses Mistral — update MODEL to your Gemma3 Modelfile name |
| `ingest_pdfs.py` | Keep but update | Same — update MODEL |
| `repo_ingest.py` | Optional | Only useful if you want repo docs in KB |
| `analyze_experiments.py` | Superseded | Replaced by `analyze_session.py` |
| `build_code_context.py` | Optional | Keep if useful for context building |
| `normalize_picarx_experiments.py` | Obsolete | Normalization pipeline abandoned |
| `ingest_picarx_experiments.py` | Obsolete | Replaced by `analyze_session.py` |
| `query_kb.py` | Optional | Keep if useful |

## Files to remove from KB before re-ingesting

These files contain stale information that will confuse the KB:

- Any summaries of `track_navigator.py` or `track_monitor.py`
- Any summaries of `picar_agent_v7.py` or `picar_agent_v8.py`
- Any summaries of `picar_navigator.py` (the original vision navigator)
- Experiment summaries from track sessions (they reference obsolete architecture)

## New files to ingest into KB

| File | Type | Notes |
|------|------|-------|
| `pi/picar_agent.py` | Code | Current Pi agent |
| `navigator/roam_navigator.py` | Code | Free roam navigator |
| `navigator/mission_navigator.py` | Code | Mission navigator |
| `navigator/zone_calibrate.py` | Code | Zone calibration tool |
| `navigator/path_record.py` | Code | Path teaching recorder |
| `navigator/analyze_session.py` | Code | Log analyzer |
| `logs/*.json` | Experiment logs | Ingest after each session |
| `reports/diagnostic_*.md` | Analysis | Auto-generated diagnostics |
| `reports/narrative_*.md` | Narratives | Auto-generated Medium content |

## Recommended KB structure for PiCar

```
raw/
  picar-x/
    code/          ← current scripts
    logs/          ← session logs
    reports/       ← generated diagnostics and narratives
    config/        ← config.py (sanitized, no IPs)
```
