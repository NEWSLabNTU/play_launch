// ChainsView component — data-flow chains found in the running system, with
// no contract (phase 88): pick a source topic and a sink topic and see the
// paths between them over observed hops, per hop p50/p99 and whether the hop
// is carried (the stamp crossed the node: measured) or sampled (the node's
// latest input on that thread: the data flowed, the time is a lower bound),
// the sink's data age, and the budget a contract's scope path puts on the
// pair. With nothing picked, the longest carried chains and their sinks'
// ages.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';

const html = htm.bind(h);

const INTERCEPTION_OFF =
    'No chains: interception is off for this run (start with --interception on)';

// The pick survives a tab switch.
const prefs = { from: '', to: '', ageFilter: '' };

/** A duration with its unit: ms below 10 s, seconds above. */
function fmtMs(ms) {
    if (ms == null || !isFinite(ms)) return '—';
    if (ms < 1) return ms.toFixed(2) + ' ms';
    if (ms < 10) return ms.toFixed(1) + ' ms';
    if (ms < 10000) return Math.round(ms) + ' ms';
    return (ms / 1000).toFixed(1) + ' s';
}

function pct(x) {
    return x == null ? '—' : Math.round(x * 100) + '%';
}

/** Poll a JSON endpoint every `ms` while mounted. A 409 means interception
 * is off for this run, which is a state, not an error. */
function usePoll(url, ms) {
    const [state, setState] = useState({ data: null, off: false, error: null });
    useEffect(() => {
        if (!url) { setState({ data: null, off: false, error: null }); return undefined; }
        let alive = true;
        const load = () => fetch(url)
            .then(async r => {
                if (!alive) return;
                if (r.status === 409) { setState({ data: null, off: true, error: null }); return; }
                if (!r.ok) { setState(s => ({ ...s, error: 'HTTP ' + r.status })); return; }
                const data = await r.json();
                if (alive) setState({ data, off: false, error: null });
            })
            .catch(e => { if (alive) setState(s => ({ ...s, error: String(e) })); });
        load();
        const id = setInterval(load, ms);
        return () => { alive = false; clearInterval(id); };
    }, [url, ms]);
    return state;
}

function KindBadge({ kind }) {
    const title = kind === 'carried'
        ? 'The stamp crossed the node: take to publish, measured'
        : "The node's latest input on the publishing thread: the data flowed, "
          + 'the trigger is unknown, and the time (since its latest take) is a lower bound';
    return html`<span class="chain-kind chain-kind-${kind}" title=${title}>${kind}</span>`;
}

function AgeLine({ age }) {
    if (!age) {
        return html`<div class="chain-age ent-muted">No data age for this topic yet (needs 20 publishes with an upstream).</div>`;
    }
    return html`
        <div class="chain-age">
            <span>Data age at the sink</span>
            <b>p50 ${fmtMs(age.p50_ms)}</b>
            <span class="ent-sep">·</span>
            <b>p99 ${fmtMs(age.p99_ms)}</b>
            <span class="ent-sep">·</span>
            <span>max ${fmtMs(age.max_ms)}</span>
            <span class="ent-sep">·</span>
            <span title="Most common topic the data began on">from <b>${age.source}</b> (${pct(age.source_share)})</span>
            ${age.estimated_share > 0 && html`
                <span class="ent-sep">·</span>
                <span class="chain-estimated" title="Publishes whose age passed through a sampled hop, or an input whose publish was not seen">
                    ${pct(age.estimated_share)} estimated</span>`}
        </div>
    `;
}

function BudgetBar({ path, budget }) {
    if (!budget) return null;
    const b = budget.max_latency_ms;
    const scale = Math.max(b, path.p99_ms) * 1.05;
    const w = (v) => Math.min(100, (v / scale) * 100) + '%';
    const over = path.p99_ms > b;
    return html`
        <div class="chain-budget" title=${'Contract scope path ' + budget.name + ': ' + fmtMs(b)}>
            <div class="chain-budget-p99 ${over ? 'chain-over' : ''}" style=${{ width: w(path.p99_ms) }}></div>
            <div class="chain-budget-p50" style=${{ width: w(path.p50_ms) }}></div>
            <div class="chain-budget-line" style=${{ left: w(b) }}></div>
        </div>
    `;
}

function PathCard({ path, index, budget }) {
    return html`
        <div class="chain-path">
            <div class="chain-path-head">
                <b>Path ${index + 1}</b>
                <span>${path.hops.length} hop${path.hops.length > 1 ? 's' : ''}</span>
                <span class="ent-sep">·</span>
                <span>${path.carried_hops} carried, ${path.sampled_hops} sampled</span>
                <span class="ent-sep">·</span>
                <span title="Sum of the hops' medians">Σp50 <b>${fmtMs(path.p50_ms)}</b></span>
                <span title="Sum of the hops' 99th percentiles">Σp99 <b>${fmtMs(path.p99_ms)}</b></span>
                ${budget && html`<span class=${path.p99_ms > budget.max_latency_ms ? 'chain-over-text' : ''}>
                    budget ${fmtMs(budget.max_latency_ms)} (${budget.name})</span>`}
            </div>
            <${BudgetBar} path=${path} budget=${budget} />
            <table class="ent-table chain-hops">
                <thead><tr>
                    <th>From</th><th>Node</th><th>To</th><th>Hop</th>
                    <th class="ent-num">p50</th><th class="ent-num">p99</th>
                    <th class="ent-num">Samples</th><th class="ent-num">Share</th>
                </tr></thead>
                <tbody>
                    ${path.hops.map((hp, i) => html`
                        <tr class="ent-row" key=${i}>
                            <td class="ent-topic">${hp.from}</td>
                            <td class="ent-node">${hp.node}</td>
                            <td class="ent-topic">${hp.to}</td>
                            <td><${KindBadge} kind=${hp.kind} /></td>
                            <td class="ent-num">${fmtMs(hp.p50_ms)}</td>
                            <td class="ent-num">${fmtMs(hp.p99_ms)}</td>
                            <td class="ent-num">${hp.samples}</td>
                            <td class="ent-num" title=${hp.share == null ? '' : "Of the node's publishes on this output"}>${pct(hp.share)}</td>
                        </tr>
                    `)}
                </tbody>
            </table>
        </div>
    `;
}

function Paths({ from, to }) {
    const url = from && to
        ? '/api/chains/paths?from=' + encodeURIComponent(from) + '&to=' + encodeURIComponent(to)
        : null;
    const { data, error } = usePoll(url, 2000);
    if (!url) return null;
    if (error && !data) return html`<div class="ent-empty">${error}</div>`;
    if (!data) return html`<div class="ent-muted">loading…</div>`;
    return html`
        <div class="chain-result">
            <${AgeLine} age=${data.sink_age} />
            ${data.paths.length > 0 && html`<div class="ent-muted chain-note">Hop times are inside each node;
                the sink's age also counts the transport between them.</div>`}
            ${data.paths.length === 0 ? html`
                <div class="ent-empty">No observed path from ${from} to ${to}. Hops need 20 observations;
                    a node that restamps with no input on its publishing thread starts a new chain.</div>
            ` : data.paths.map((p, i) => html`<${PathCard} key=${i} path=${p} index=${i} budget=${data.budget} />`)}
            ${data.truncated && html`<div class="ent-muted">The search hit its bound; longer paths may be missing.</div>`}
        </div>
    `;
}

export function ChainsView() {
    const { data: snap, off, error } = usePoll('/api/chains', 2000);
    const [from, setFromRaw] = useState(prefs.from);
    const [to, setToRaw] = useState(prefs.to);
    const [ageFilter, setAgeFilterRaw] = useState(prefs.ageFilter);
    const setFrom = (v) => { prefs.from = v; setFromRaw(v); };
    const setTo = (v) => { prefs.to = v; setToRaw(v); };
    const setAgeFilter = (v) => { prefs.ageFilter = v; setAgeFilterRaw(v); };

    const topics = useMemo(() => {
        if (!snap) return { sources: [], sinks: [] };
        const src = new Set();
        const dst = new Set();
        for (const e of [...snap.edges, ...snap.sampled]) { src.add(e.from); dst.add(e.to); }
        return { sources: [...src].sort(), sinks: [...dst].sort() };
    }, [snap]);
    const ages = useMemo(() => {
        if (!snap) return new Map();
        return new Map(snap.ages.map(a => [a.topic, a]));
    }, [snap]);

    if (off) return html`<div class="entities-view"><div class="ent-empty">${INTERCEPTION_OFF}</div></div>`;

    const pick = (f, t) => { setFrom(f); setTo(t); };
    const shownAges = snap
        ? snap.ages
            .filter(a => !ageFilter || a.topic.includes(ageFilter) || a.source.includes(ageFilter))
            .sort((a, b) => (b.p99_ms ?? 0) - (a.p99_ms ?? 0))
            .slice(0, 50)
        : [];

    return html`
        <div class="entities-view chains-view">
            <div class="ent-summary">
                ${snap ? html`
                    <b>${snap.edges.length}</b> carried hops
                    <span class="ent-sep">·</span>
                    <b>${snap.sampled.length}</b> sampled
                    <span class="ent-sep">·</span>
                    <b>${snap.chains.length}</b> chains
                    <span class="ent-sep">·</span>
                    <b>${snap.ages.length}</b> topics with a data age
                ` : html`<span class="ent-muted">${error || 'waiting for first snapshot…'}</span>`}
            </div>
            <div class="ent-controls">
                <input type="text" class="ent-search" list="chain-sources" placeholder="Source topic..."
                    value=${from} onInput=${(e) => setFrom(e.target.value)} />
                <span class="ent-muted">→</span>
                <input type="text" class="ent-search" list="chain-sinks" placeholder="Sink topic..."
                    value=${to} onInput=${(e) => setTo(e.target.value)} />
                ${(from || to) && html`<button class="chain-clear" onClick=${() => pick('', '')}>Clear</button>`}
                <datalist id="chain-sources">${topics.sources.map(t => html`<option value=${t} />`)}</datalist>
                <datalist id="chain-sinks">${topics.sinks.map(t => html`<option value=${t} />`)}</datalist>
            </div>

            ${from && to ? html`<${Paths} from=${from} to=${to} />` : snap && html`
                <h3 class="chain-heading">Longest carried chains</h3>
                ${snap.chains.length === 0 ? html`<div class="ent-empty">No chain yet: carried hops need 20 observations.</div>` : html`
                    <table class="ent-table">
                        <thead><tr>
                            <th>Topics</th><th class="ent-num">Hops</th><th class="ent-num">Σp50</th>
                            <th class="ent-num">Sink age p50</th><th class="ent-num">p99</th>
                        </tr></thead>
                        <tbody>
                            ${snap.chains.map((c, i) => {
                                const sink = c.topics[c.topics.length - 1];
                                const a = ages.get(sink);
                                return html`
                                    <tr class="ent-row chain-pick" key=${i} title="Show the paths from source to sink"
                                        onClick=${() => pick(c.topics[0], sink)}>
                                        <td class="chain-topics">${c.topics.join(' → ')}</td>
                                        <td class="ent-num">${c.nodes.length}</td>
                                        <td class="ent-num">${fmtMs(c.p50_ms)}</td>
                                        <td class="ent-num">${fmtMs(a && a.p50_ms)}</td>
                                        <td class="ent-num">${fmtMs(a && a.p99_ms)}</td>
                                    </tr>`;
                            })}
                        </tbody>
                    </table>`}
                <h3 class="chain-heading">Data age, oldest first</h3>
                <div class="ent-controls">
                    <input type="text" class="ent-search" placeholder="Filter topic / source..."
                        value=${ageFilter} onInput=${(e) => setAgeFilter(e.target.value)} />
                </div>
                <table class="ent-table">
                    <thead><tr>
                        <th>Topic</th><th>Data began on</th>
                        <th class="ent-num">p50</th><th class="ent-num">p99</th>
                        <th class="ent-num">Estimated</th><th class="ent-num">Samples</th>
                    </tr></thead>
                    <tbody>
                        ${shownAges.map(a => html`
                            <tr class="ent-row chain-pick" key=${a.topic} title="Show the paths from the source to this topic"
                                onClick=${() => pick(a.source, a.topic)}>
                                <td class="ent-topic">${a.topic}</td>
                                <td class="ent-topic">${a.source} <span class="ent-sub">${pct(a.source_share)}</span></td>
                                <td class="ent-num">${fmtMs(a.p50_ms)}</td>
                                <td class="ent-num">${fmtMs(a.p99_ms)}</td>
                                <td class="ent-num">${pct(a.estimated_share)}</td>
                                <td class="ent-num">${a.samples}</td>
                            </tr>`)}
                    </tbody>
                </table>
            `}
        </div>
    `;
}
