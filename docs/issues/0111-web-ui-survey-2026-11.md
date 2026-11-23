---
id: 111
title: "Web UI: mutations open to any page, stale header figures, lint skips components"
status: open
type: defect
severity: high
---

# 0111 - Web UI survey, 2026-11

**Repo:** `play_launch` 0.41.4
**Affects:** the web UI (`src/play_launch/src/web/`)
**Found:** read-only survey, 2026-11-23. Plan: phase 105.

Paths are relative to `src/play_launch/src/web/`. Severity / effort.

## Defects

1. **Cross-site requests (high / S).** The body-less POSTs (`mod.rs`
   217-258) are CORS simple requests any open web page can send; CORS
   (`mod.rs` 180-206) only hides the answer. No `Origin` or `Host` check, so
   DNS rebinding reads every GET too. No authentication on any bind.
2. **Header node count stale (high / S).** `healthSummary` is written only
   by `fetchHealth` (`sse.js` 56, 74; `NodeCard.js` 178); `applyStateEvent`
   never updates it, and the `data.summary` branch (`store.js` 76) is dead,
   since the health snapshot has no `summary` field. The live `failed`
   badge beside it (`Header.js` 18) disagrees.
3. **Header diagnostics badges (high / S)** read `diagnostics`, which only
   `DiagnosticsView.js` 142 writes, while that page is open.
4. **ESLint skips `components/` (med / S).** `scripts/lint-inline-js.js`
   16 reads the directory non-recursively. `tsconfig.json` has
   `strict: false`.
5. **Connection cap (med / M).** Two global streams, plus Entities, plus
   stderr (always on, `tab === 'stderr' || true`, `RightPanel.js` 118):
   two tabs reach the browser's six connections per origin and polls hang.
6. **Activity stale (med / S).** No event carries `stderr_last_modified`;
   the hidden NodeList keeps its 3 s poll (`NodeList.js` 127; `app.js`
   82-85).
7. **Undefined CSS variables (med / S).** `--border` (`launch-tree.css`
   17, 41, 353, 403) and `--hover-bg` (49, 75, 100, 133, 433), no fallback.
8. **Badge poll fetches the whole health snapshot (med / S)** every 5 s
   for three counts (`Header.js` 153-165).
9. **Log tab under a flood (med / M).** A copy of up to 5000 lines per
   message, index keys, no batching (`LogTab.js` 56-131).
10. **Graph at scale (med / M, unmeasured).** ELK on the main thread
    (`graph-layout.js` 6); each state event clones the `nodes` map
    (`store.js` 229).
11. **No URL routing (med / S).** A reload loses the page and selection.
12. **Accessibility (med / M).** No `aria-`/`role`; at least 12 clickable
    elements without keyboard access; one focus style; mouse-only resizer
    (`PanelResizer.js` 26).
13. **Silent failures (low / S).** Bulk actions ignore `res.ok`
    (`NodeList.js` 337, 346); `alert`/`confirm`; `.catch(()=>{})` in
    `Header.js` and `HealthView.js`.
14. **Duplication (low / S).** Probe-then-stream twice (`EntitiesView.js`
    285-335, `HealthView.js` 615-660); `usePoll` only in `ChainsView.js`;
    Escape handler three times; two identical activity icons; `htm.bind` in
    19 files.
15. **Dead code (low / S).** `escapeHtml` (`InfoTab.js` 8), the
    `data.summary` branch, `/static/*path` (`mod.rs` 212), restart-all
    with no caller, a stale 409 comment (`sse.js` 91).
16. **Small (low).** `::` not treated as exposed (string compare to
    `0.0.0.0`); theme flash on load; three mobile media rules; no JS tests;
    no check that the generated bindings are current.

No XSS found: htm escapes interpolated text, log lines included.
