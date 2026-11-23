// ELK's layout in a Web Worker (phase 105 W6), so a large graph's layout
// does not freeze the page. The vendored bundle, constructed in a worker
// (no `document`), installs ELK's own worker-side message handler on
// `self`; graph-layout.js talks to it through ELK's `workerFactory`.
import ELK from './vendor/elk.bundled.esm.js';

new ELK();
