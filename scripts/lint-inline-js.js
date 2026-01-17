#!/usr/bin/env node

const { ESLint } = require('eslint');
const path = require('path');

const JS_FILE = path.join(__dirname, '../src/play_launch/src/web/assets/script.js');

async function main() {
    const fix = process.argv.includes('--fix');

    try {
        // Run ESLint on script.js
        const eslint = new ESLint({ fix });
        const results = await eslint.lintFiles([JS_FILE]);

        if (fix) {
            await ESLint.outputFixes(results);
            console.log('Auto-fix applied to script.js');
        }

        // Format results
        const formatter = await eslint.loadFormatter('stylish');
        const resultText = formatter.format(results);

        if (resultText) {
            console.log('JavaScript Lint Results:');
            console.log(resultText);
        } else {
            console.log('✓ JavaScript: No issues found');
        }

        // Exit with error code if there are errors
        const hasErrors = results.some(result => result.errorCount > 0);
        if (hasErrors) {
            process.exit(1);
        }
    } catch (error) {
        console.error('Error during JavaScript linting:', error.message);
        process.exit(1);
    }
}

main();
