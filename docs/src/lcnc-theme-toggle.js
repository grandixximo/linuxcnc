// LinuxCNC docs theme toggle: persistence layer only.
//
// The Legacy / Modern switch is the #lcnc-mode checkbox injected into
// the page by docinfo-footer.html (asciidoctor) or the index*.tmpl /
// gcode.html static templates.  CSS in lcnc-overrides.css keys off
// `body:has(#lcnc-mode:checked)`, so the toggle works on the current
// page with JS disabled.
//
// This script only persists the choice across page navigation: read
// localStorage on load and set the checkbox state, write localStorage
// on each change.  No body-class manipulation, no inline styles.

(function () {
  function sync() {
    var input = document.getElementById('lcnc-mode');
    if (!input) return;
    try {
      input.checked = (localStorage.getItem('lcnc-theme') === 'legacy');
    } catch (e) {}
    input.addEventListener('change', function () {
      try {
        localStorage.setItem('lcnc-theme', input.checked ? 'legacy' : 'modern');
      } catch (e) {}
    });
  }
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', sync);
  } else {
    sync();
  }
})();
