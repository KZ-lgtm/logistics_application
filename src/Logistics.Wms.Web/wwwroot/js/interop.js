// deep link handed over by 404.html (GitHub Pages has no server-side fallback to index.html)
(function () {
    var q = new URLSearchParams(window.location.search), p = q.get('p');
    if (p !== null) {
        var base = document.querySelector('base').getAttribute('href');
        window.history.replaceState(null, '', base + p.replace(/^\//, '') + window.location.hash);
    }
})();

// boot screen in the UI language (the same rule as the app: saved choice, else the browser's language)
(function () {
    var v = null; try { v = localStorage.getItem('logistics-wms/lang'); } catch (e) { }
    var l = (navigator.languages && navigator.languages[0]) || navigator.language || '';
    var zh = v ? v === 'zh-Hant' : /^zh-(TW|HK|MO|Hant)/i.test(l);
    document.documentElement.lang = zh ? 'zh-Hant-TW' : 'en';
    if (!zh) return;
    var t = document.querySelector('.boot-text'), n = document.querySelector('.boot-note');
    if (t) t.textContent = '正在載入 WMS'; if (n) n.textContent = '.NET 執行環境與 WMS 在您的瀏覽器中執行；第一次載入約需下載 10 MB。';
    document.title = '物流 WMS';
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
    // the live 3D cell (twin/index.html, in an iframe) reaches the WMS through this bridge in the parent page
    registerTwin: function (ref) {
        window.wmsTwinBridge = { invoke: function () { return ref.invokeMethodAsync.apply(ref, arguments); } };
    },
    unregisterTwin: function () { delete window.wmsTwinBridge; },
    // the 3D cell keeps running when its page is not shown (no rendering then), and can point at an order
    twinCall: function (name, arg) {
        var f = document.getElementById('wms-twin'), w = f && f.contentWindow;
        if (w && typeof w[name] === 'function') w[name](arg); else if (f) (f.pending = f.pending || {})[name] = arg;
    },
    twinVisible: function (v) { window.wms.twinCall('wmsTwinVisible', v); },
    twinFocus: function (id) { window.wms.twinCall('wmsTwinFocus', id); },
    // UI language: the saved choice, else the browser's (Traditional Chinese for zh-TW / zh-HK / zh-MO / zh-Hant)
    language: function (key) {
        var v = null; try { v = localStorage.getItem(key); } catch (e) { }
        if (v) return v;
        var l = (navigator.languages && navigator.languages[0]) || navigator.language || '';
        return /^zh-(TW|HK|MO|Hant)/i.test(l) ? 'zh-Hant' : 'en';
    },
    setLanguage: function (key, v) {
        try { localStorage.setItem(key, v); } catch (e) { }
        document.documentElement.lang = v === 'zh-Hant' ? 'zh-Hant-TW' : 'en';
    },
    scrollTo: function (id) { var e = document.getElementById(decodeURIComponent(id)); if (e) e.scrollIntoView(); },
    storageEstimate: function () {
        var n = 0;
        try { for (var i = 0; i < localStorage.length; i++) { var k = localStorage.key(i); n += k.length + (localStorage.getItem(k) || '').length; } } catch (e) { }
        return n;
    }
};
