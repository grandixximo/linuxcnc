// LinuxCNC docs theme toggle.
//
// Top-right Legacy / Modern button that flips body.lcnc-legacy
// <-> body.lcnc-modern.  Choice persists in localStorage and
// applies across every page that loads this script.
//
// Works for both asciidoctor-generated pages (via docinfo.html) and
// the hand-written index.tmpl / gcode.html.  Requires a <link
// id="lcnc-overrides-link"> already present in the page <head>.
//
// (A previous version of this file rewrote the lcnc-overrides.css
// <link> href at runtime as a cache-buster.  Removed because the
// re-fetch caused a flash of unstyled content on every reload.
// During development hit Ctrl-F5 after editing the CSS.)

(function () {
  var THEMES = ['legacy', 'modern'];
  function apply(theme) {
    var b = document.body;
    // Asciidoctor pages (body.article / body.book / body.manpage) get
    // lcnc-{legacy,modern}; hand-written index.tmpl / gcode.html get
    // lcnc-static-{legacy,modern}.  Keeping the two namespaces apart
    // lets each set of CSS rules target exactly the markup it was
    // written for, with no cross-contamination.
    var isAsciidoc = b.classList.contains('article') ||
                     b.classList.contains('book') ||
                     b.classList.contains('manpage');
    var prefix = isAsciidoc ? 'lcnc-' : 'lcnc-static-';
    // Scrub BOTH namespaces' classes, not just the active one.  Otherwise
    // pages that were toggled by an older version of this script (which
    // misclassified manpages as static) end up with a stale lcnc-static-*
    // class clinging on, masking the real theme rules.
    ['lcnc-', 'lcnc-static-'].forEach(function (p) {
      THEMES.forEach(function (t) { b.classList.remove(p + t); });
    });
    b.classList.add(prefix + theme);
    var btn = document.getElementById('lcnc-theme-toggle');
    if (btn) btn.textContent = theme === 'modern' ? 'Modern' : 'Legacy';
    try { localStorage.setItem('lcnc-theme', theme); } catch (e) {}
    // FOUC guard from <head> can go away now that the real theme rules
    // are in effect (otherwise it would pin dark even after user picks
    // Legacy).
    var g = document.getElementById('lcnc-fouc-guard');
    if (g) g.remove();
  }
  function init() {
    var btn = document.createElement('button');
    btn.id = 'lcnc-theme-toggle';
    btn.type = 'button';
    btn.style.cssText = 'position:fixed;top:8px;right:8px;z-index:9999;' +
      'padding:4px 10px;font:12px sans-serif;border:1px solid #999;' +
      'background:#fff;color:#333;border-radius:3px;cursor:pointer;' +
      'box-shadow:0 1px 3px rgba(0,0,0,0.2)';
    btn.title = 'Switch between LinuxCNC legacy and asciidoctor modern themes';
    btn.addEventListener('click', function () {
      var cur = 'legacy';
      try { cur = localStorage.getItem('lcnc-theme') || 'legacy'; } catch (e) {}
      apply(cur === 'modern' ? 'legacy' : 'modern');
    });
    document.body.appendChild(btn);
    var saved = null;
    try { saved = localStorage.getItem('lcnc-theme'); } catch (e) {}
    apply(saved === 'modern' ? 'modern' : 'legacy');
  }
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
