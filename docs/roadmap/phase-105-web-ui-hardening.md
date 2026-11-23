# Phase 105: Web UI hardening and modernization

**Status:** In progress (2026-11-23).
**Fixes:** issue #0111.
**Prior work:** phase 20 (Preact + htm + SSE), phases 50 and 53 (issue
#0006, the 2026-07 audit in `docs/audit/architecture-2026-07.md`).

## Why

The 2026-07 audit's web UI gaps were closed by phases 50 and 53. Since then
the Health, Entities, Chains, Diagnostics and Launch pages were added by
feature work, each with its own way of staying fresh, and no pass looked at
the UI as a whole. A read-only survey on 2026-11-23 found one security
hole, three header figures that are wrong most of the time, a lint gate
that skips most of the code, and seven SSE endpoints and twelve polls doing
what one stream could. The findings, each with its file and line, are in
issue #0111.

Paths below are relative to `src/play_launch/src/web/`.

## Freshness today

SSE (7 endpoints, 8 client streams): `state/updates` and `metrics/system`
always; `entities/stream`, `health/issues/stream`, `diagnostics/stream`,
the two log streams and `metrics/node` per page. Polls (12): two reconnect
probes (2 s), `/api/graph` 10 s, `/api/nodes` 3 s under activity sort,
`/api/interception/events` 5 s, `/api/health/issues` 5 s, `/api/startup`
2-30 s, the DDS report 10 s, diagnostics history 2 s, `/api/chains` and
`/api/chains/paths` 2 s, and two probe retries 3 s. Server side, the log
tail polls files every 500 ms and the diagnostics stream re-sends the full
list every second. Nothing pauses on a hidden tab.

## Work items

### W1 - Mutations only from our own page (security)

**Done (2026-11-23).** `web/origin_guard.rs`; tests
`tests/tests/web_origin_guard.rs` (the refusal test fails with the
middleware removed). The token is kept in an `HttpOnly; SameSite=Strict`
cookie rather than `sessionStorage`, so `EventSource` streams carry it and
no page script can read it.

Every body-less POST (`start`, `stop`, `restart`, `load`, `unload`, the
bulk actions, `respawn`, `auto-load`; `mod.rs` routes) is a CORS "simple
request": any web page the user has open can send it, and CORS only hides
the answer. Nothing checks `Origin` or `Host`, so a DNS-rebinding page can
also read every GET.

- A middleware on every route: `Host` must name the bound address,
  `localhost`, or a loopback address (with the port) unless bound to an
  unspecified address; a non-GET/HEAD request with an `Origin` must carry
  one the same rule accepts. Refused with 403 and a message naming the
  rule.
- Treat `::` like `0.0.0.0` (today a string compare).
- Optional token when bound to a non-loopback address (`web_ui.token` /
  `--web-token`): required on every request, accepted from a header or a
  `?token=` that the page then keeps in `sessionStorage`. Default: none, so
  loopback use is unchanged.
- Tests: a foreign `Origin` POST is refused and does nothing; a rebinding
  `Host` GET is refused; the page's own requests pass.

### W2 - The gates see the code

**Done (2026-11-23), except `strict`.** ESLint walks every file under
`js/` and fails on any finding; 526 surfaced, 410 of them `curly` (turned
off: a one-line guard is this code's style) and 76 deliberate `== null`
(allowed). `just check-bindings` regenerates the bindings and fails on a
difference (checked both ways). The health, startup, entities, chains,
graph and diagnostics payloads are typed (19 to 71 binding files, large
integers as `number`, which is what JSON carries), and typing them found a
defect at once: the header's diagnostics badges compared levels against
`'OK'`/`'WARNING'`/... while the payload says `'Ok'`/`'Warning'`/..., so
they never showed. `strict` per file is left to each later wave, for the
files it touches.

- `scripts/lint-inline-js.js` reads `js/` non-recursively, so ESLint never
  sees `components/` (~6k lines). Walk recursively; fix what it finds.
- `generate-bindings` exists but nothing checks the committed TypeScript
  bindings are current. A `just check` step regenerates into a temporary
  directory and diffs.
- Type the payloads added since phase 20 (health snapshot, entities,
  chains, startup, graph, diagnostics) with `ts-rs`, and turn on `strict`
  per file as each is cleaned.

### W3 - The header tells the truth

- `healthSummary` is written only on SSE connect, `refresh` and after a
  card action; state events never touch it, and the `data.summary` branch
  in `store.js` is dead (the snapshot has no such field). The node count
  freezes through startup and crashes while the `failed` badge beside it
  is live. Derive the counts from the `nodes` store, or push them.
- The diagnostics badges read data only the Diagnostics page writes.
- The issue badges poll the whole health snapshot every 5 s for three
  numbers.
- All three ride W4's stream as small count events.

### W4 - One stream, shared hooks

- One multiplexed SSE connection with named events (`state`, `metrics`,
  `health_counts`, `diagnostics_counts`, and page topics subscribed by
  query) replaces the two global streams; pages that need a large snapshot
  keep their own stream but join through one hook. Goal: at most two
  connections per tab, under the browser's six per origin with two tabs
  open (today two tabs hang).
- `useSnapshotStream(url)` and `usePoll(url, ms)` shared hooks replace the
  copied probe-then-stream code (Entities, Health) and the hand-rolled
  intervals (Header, Health, Diagnostics, Chains). Both pause on
  `document.hidden`.
- Activity (`stderr_last_modified`) rides the stream, retiring the 3 s
  `/api/nodes` poll (the item deferred in phase 53). The hidden NodeList
  stops polling.
- The stderr stream stops being always-on (`|| true` in `RightPanel.js`).
- Diagnostics stream sends changes, not the full list each second.

### W5 - Correct and accessible

- Define `--border` and `--hover-bg` (undefined, no fallback: launch-tree
  borders and hover never draw) and the other undefined tokens; move
  hardcoded colours to tokens; the graph styles take the palette from CSS
  instead of repeating it in JS.
- Theme applied before first paint (inline script in `index.html`).
- Hash routing `#/<view>/<member>`: reload keeps the page and selection,
  and a link opens it.
- Keyboard and ARIA: clickable rows become buttons or get `role`,
  `tabindex` and key handlers; a visible focus style; the panel resizer
  takes arrow keys.
- Errors: bulk actions check `res.ok`; `alert`/`confirm` become an inline
  toast and confirm; no silent `.catch(()=>{})`.
- Mobile width: the side panel becomes full width under 768 px.

### W6 - Scale

- Log tab: batch appends per animation frame, stable keys, a ring buffer
  instead of a copy per line; virtualize the list.
- State events update one entry without cloning the whole `nodes` map on
  every event in a burst.
- ELK layout in a Web Worker (`workerUrl`).

### W7 - Cleanup

Delete `escapeHtml`, the `/static/*path` route, the unused restart-all
route, the stale 409 comment; merge the duplicated Escape handlers,
node-detail fetches and the two activity icons; one `html` export instead
of `htm.bind` in 19 files. Bump the vendored Preact, signals and elkjs if
newer releases pass the suite.

## Not in scope

A bundler (a localhost tool with 13 stylesheets and ~35 modules gains
nothing a `modulepreload` list cannot give), and a framework change.

## Acceptance

- W1: the three refusal tests pass and fail with the middleware removed.
- W2: `just check` lints every file under `js/` and fails on a stale
  binding.
- W3: on a launch where a node crashes, the header count changes with no
  page action.
- W4: two tabs of the UI on Autoware load every page; connection count per
  tab at most two plus page streams.
- Each wave: `just check` and `just test-all` pass.
