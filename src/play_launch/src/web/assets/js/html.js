// The one `html` tagged template (htm bound to Preact's h). Each module
// used to bind its own copy (phase 105 W7).
import { h } from './vendor/preact.module.js';
import htm from './vendor/htm.module.js';

export const html = htm.bind(h);
