#!/usr/bin/env node

const fs = require('fs');
const path = require('path');
const { ESLint } = require('eslint');

const HTML_FILE = path.join(__dirname, '../src/play_launch/src/web/assets/index.html');
const TEMP_JS_FILE = path.join(__dirname, '../.tmp-extracted.js');

// Extract JavaScript from HTML
function extractJavaScript(htmlContent) {
    const scriptRegex = /<script[^>]*>([\s\S]*?)<\/script>/gi;
    const matches = [];
    let match;

    while ((match = scriptRegex.exec(htmlContent)) !== null) {
        const content = match[1].trim();
        if (content && !content.includes('src=') && !content.includes('unpkg.com')) {
            matches.push(content);
        }
    }

    return matches.join('\n\n');
}

async function main() {
    const fix = process.argv.includes('--fix');

    try {
        // Read HTML file
        const htmlContent = fs.readFileSync(HTML_FILE, 'utf8');

        // Extract JavaScript
        const jsContent = extractJavaScript(htmlContent);

        if (!jsContent) {
            console.log('No inline JavaScript found.');
            return;
        }

        // Write temporary JS file
        fs.writeFileSync(TEMP_JS_FILE, jsContent);

        // Run ESLint
        const eslint = new ESLint({
            fix,
            overrideConfig: {
                ignorePatterns: ['!.tmp-extracted.js']
            }
        });
        const results = await eslint.lintFiles([TEMP_JS_FILE]);

        if (fix) {
            await ESLint.outputFixes(results);
            // TODO: Optionally write fixed content back to HTML
            console.log('Auto-fix applied to temporary file. Manual integration needed.');
        }

        // Format results
        const formatter = await eslint.loadFormatter('stylish');
        const resultText = formatter.format(results);

        // Clean up temp file
        fs.unlinkSync(TEMP_JS_FILE);

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
        // Clean up on error
        if (fs.existsSync(TEMP_JS_FILE)) {
            fs.unlinkSync(TEMP_JS_FILE);
        }
        console.error('Error during JavaScript linting:', error.message);
        process.exit(1);
    }
}

main();
