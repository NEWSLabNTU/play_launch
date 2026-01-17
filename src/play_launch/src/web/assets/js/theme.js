// Theme management
function getSystemTheme() {
    return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
}

function setTheme(theme) {
    document.documentElement.setAttribute('data-theme', theme);
    localStorage.setItem('theme', theme);
    updateThemeIcon();
}

function toggleTheme() {
    const current = document.documentElement.getAttribute('data-theme');
    const next = current === 'dark' ? 'light' : 'dark';
    setTheme(next);
}

function updateThemeIcon() {
    const theme = document.documentElement.getAttribute('data-theme');
    document.getElementById('theme-icon').textContent = theme === 'dark' ? '☀' : '🌙';
}

// Initialize theme
const savedTheme = localStorage.getItem('theme') || getSystemTheme();
setTheme(savedTheme);

// Watch for system theme changes
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
    if (!localStorage.getItem('theme')) {
        setTheme(e.matches ? 'dark' : 'light');
    }
});

