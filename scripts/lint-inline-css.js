#!/usr/bin/env node

const stylelint = require('stylelint');
const path = require('path');
const fs = require('fs');

const CSS_DIR = path.join(__dirname, '../src/play_launch/src/web/assets/css');

async function main() {
    try {
        // Get all .css files in the css directory
        const cssFiles = fs.readdirSync(CSS_DIR)
            .filter(file => file.endsWith('.css'))
            .map(file => path.join(CSS_DIR, file));

        if (cssFiles.length === 0) {
            console.log('No CSS files found.');
            return;
        }

        // Run stylelint on all CSS files
        const result = await stylelint.lint({
            files: cssFiles,
            formatter: 'string'
        });

        if (result.output) {
            console.log('CSS Lint Results:');
            console.log(result.output);
        } else {
            console.log('✓ CSS: No issues found');
        }

        // Exit with error code if there are errors
        if (result.errored) {
            process.exit(1);
        }
    } catch (error) {
        console.error('Error during CSS linting:', error.message);
        process.exit(1);
    }
}

main();
