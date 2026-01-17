#!/usr/bin/env node

const { ESLint } = require('eslint');
const path = require('path');
const fs = require('fs');

const JS_DIR = path.join(__dirname, '../src/play_launch/src/web/assets/js');

async function main() {
    const fix = process.argv.includes('--fix');

    try {
        // Get all .js files in the js directory
        const jsFiles = fs.readdirSync(JS_DIR)
            .filter(file => file.endsWith('.js'))
            .map(file => path.join(JS_DIR, file));

        if (jsFiles.length === 0) {
            console.log('No JavaScript files found.');
            return;
        }

        // Run ESLint on all JS files
        const eslint = new ESLint({ fix });
        const results = await eslint.lintFiles(jsFiles);

        if (fix) {
            await ESLint.outputFixes(results);
            console.log('Auto-fix applied to JavaScript files');
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
