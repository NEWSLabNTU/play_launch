// ESLint flat config for the web UI's JavaScript (scripts/lint-inline-js.js).
// The rules are the old .eslintrc.json's, unchanged.
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
        rules: {
            'no-unused-vars': 'warn',
            'no-undef': 'warn',
            'no-console': 'off',
            'semi': ['error', 'always'],
            'quotes': ['warn', 'single', { avoidEscape: true, allowTemplateLiterals: true }],
            'indent': 'off',
            'no-var': 'warn',
            'prefer-const': 'warn',
            'no-multiple-empty-lines': ['warn', { max: 1 }],
            'eqeqeq': ['warn', 'always'],
            'curly': ['warn', 'all'],
        },
    },
];
