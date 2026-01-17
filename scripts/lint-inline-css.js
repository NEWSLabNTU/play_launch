#!/usr/bin/env node

const fs = require('fs');
const path = require('path');
const stylelint = require('stylelint');

const HTML_FILE = path.join(__dirname, '../src/play_launch/src/web/assets/index.html');
const TEMP_CSS_FILE = path.join(__dirname, '../.tmp-extracted.css');

// Extract CSS from HTML
function extractCSS(htmlContent) {
    const styleRegex = /<style[^>]*>([\s\S]*?)<\/style>/gi;
    const matches = [];
    let match;

    while ((match = styleRegex.exec(htmlContent)) !== null) {
        const content = match[1].trim();
        if (content) {
            matches.push(content);
        }
    }

    return matches.join('\n\n');
}

async function main() {
    try {
        // Read HTML file
        const htmlContent = fs.readFileSync(HTML_FILE, 'utf8');

        // Extract CSS
        const cssContent = extractCSS(htmlContent);

        if (!cssContent) {
            console.log('No inline CSS found.');
            return;
        }

        // Write temporary CSS file
        fs.writeFileSync(TEMP_CSS_FILE, cssContent);

        // Run stylelint
        const result = await stylelint.lint({
            files: [TEMP_CSS_FILE],
            formatter: 'string'
        });

        // Clean up temp file
        fs.unlinkSync(TEMP_CSS_FILE);

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
        // Clean up on error
        if (fs.existsSync(TEMP_CSS_FILE)) {
            fs.unlinkSync(TEMP_CSS_FILE);
        }
        console.error('Error during CSS linting:', error.message);
        process.exit(1);
    }
}

main();
