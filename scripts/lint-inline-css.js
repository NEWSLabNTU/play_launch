#!/usr/bin/env node

const stylelint = require('stylelint');
const path = require('path');

const CSS_FILE = path.join(__dirname, '../src/play_launch/src/web/assets/styles.css');

async function main() {
    try {
        // Run stylelint on styles.css
        const result = await stylelint.lint({
            files: [CSS_FILE],
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
