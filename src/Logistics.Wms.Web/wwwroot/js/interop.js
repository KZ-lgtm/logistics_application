// deep link handed over by 404.html (GitHub Pages has no server-side fallback to index.html)
(function () {
    var q = new URLSearchParams(window.location.search), p = q.get('p');
    if (p !== null) {
        var base = document.querySelector('base').getAttribute('href');
        window.history.replaceState(null, '', base + p.replace(/^\//, '') + window.location.hash);
    }
})();

// loading progress from Blazor's CSS custom property
(function () {
    var t = setInterval(function () {
        var v = getComputedStyle(document.documentElement).getPropertyValue('--blazor-load-percentage');
        var el = document.querySelector('.boot-pct');
        if (!el) { clearInterval(t); return; }
        if (v) el.textContent = ' · ' + v.trim();
    }, 150);
})();

window.wms = {
    download: function (fileName, text, type) {
        var blob = new Blob([text], { type: type || 'application/json' });
        var a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = fileName;
        document.body.appendChild(a);
        a.click();
        setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 0);
    },
    storageEstimate: function () {
        var n = 0;
        try { for (var i = 0; i < localStorage.length; i++) { var k = localStorage.key(i); n += k.length + (localStorage.getItem(k) || '').length; } } catch (e) { }
        return n;
    }
};
