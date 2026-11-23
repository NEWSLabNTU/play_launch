// ESLint flat config for the web UI's JavaScript (scripts/lint-inline-js.js).
const js = require('@eslint/js');
const globals = require('globals');

module.exports = [
    js.configs.recommended,
    {
        languageOptions: {
            ecmaVersion: 2021,
            sourceType: 'module',
            globals: { ...globals.browser, ...globals.es2021 },
        },
        // Every rule is an error (phase 105 W2): a warning nothing fails on
        // is a finding nobody reads, and components/ had gathered 526 of
        // them unseen. `curly` is off: a one-line guard (`if (!x) return;`)
        // is this code's style, and 410 of the 526 were that.
        rules: {
            'no-unused-vars': ['error', { argsIgnorePattern: '^_', caughtErrors: 'none' }],
            'no-undef': 'error',
            'no-console': 'off',
            'semi': ['error', 'always'],
            'quotes': ['error', 'single', { avoidEscape: true, allowTemplateLiterals: true }],
            'indent': 'off',
            'no-var': 'error',
            'prefer-const': 'error',
            'no-multiple-empty-lines': ['error', { max: 1 }],
            // `x == null` is the one deliberate loose comparison: null or
            // undefined.
            'eqeqeq': ['error', 'always', { null: 'ignore' }],
            'curly': 'off',
        },
    },
];
