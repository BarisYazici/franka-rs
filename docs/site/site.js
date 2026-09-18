// franka-rs landing page. Progressive enhancement only: every section is
// readable and every link works with this file absent.
(function () {
  'use strict';

  var toast = document.querySelector('.toast');
  var toastTimer;
  function announce(text) {
    if (!toast) return;
    toast.textContent = text;
    toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(function () { toast.classList.remove('show'); }, 1800);
  }

  // Copy buttons on every command block. Feedback reports what actually
  // happened: a failed clipboard write says so and selects the text instead.
  function selectText(node) {
    var range = document.createRange();
    range.selectNodeContents(node);
    var sel = window.getSelection();
    sel.removeAllRanges();
    sel.addRange(range);
  }

  function setState(button, cls, label) {
    button.classList.remove('ok', 'fail');
    if (cls) button.classList.add(cls);
    button.textContent = label;
    clearTimeout(button._timer);
    button._timer = setTimeout(function () {
      button.classList.remove('ok', 'fail');
      button.textContent = 'Copy';
    }, 2000);
  }

  function copyBlock(block, button) {
    var code = block.querySelector('code');
    var text = code ? code.textContent : '';
    var write = navigator.clipboard && window.isSecureContext
      ? navigator.clipboard.writeText(text)
      : Promise.reject(new Error('clipboard unavailable'));
    write.then(function () {
      setState(button, 'ok', 'Copied');
      announce('Copied to clipboard');
    }, function () {
      if (code) selectText(code);
      setState(button, 'fail', 'Copy failed');
      announce('Copy failed. The text is selected; press Ctrl+C or Cmd+C.');
    });
  }

  Array.prototype.forEach.call(document.querySelectorAll('.cmd'), function (block) {
    var button = document.createElement('button');
    button.type = 'button';
    button.className = 'copy';
    button.textContent = 'Copy';
    button.addEventListener('click', function () { copyBlock(block, button); });
    block.appendChild(button);
  });

  // Setup selector: the nav of in-page links becomes a tab list, the target
  // sections become tab panels. Arrow keys move between tabs; the hash keeps
  // the choice shareable and lets a deep link open the right panel.
  var selector = document.querySelector('.selector');
  if (selector) {
    var links = Array.prototype.slice.call(selector.querySelectorAll('a[href^="#"]'));
    var tabs = [];
    var panels = [];

    links.forEach(function (link) {
      var panel = document.getElementById(link.getAttribute('href').slice(1));
      if (!panel) return;
      var tab = document.createElement('button');
      tab.type = 'button';
      tab.textContent = link.textContent;
      tab.setAttribute('role', 'tab');
      tab.id = 'tab-' + panel.id;
      tab.setAttribute('aria-controls', panel.id);
      panel.setAttribute('role', 'tabpanel');
      panel.setAttribute('aria-labelledby', tab.id);
      panel.tabIndex = 0;
      selector.replaceChild(tab, link);
      tabs.push(tab);
      panels.push(panel);
    });

    if (tabs.length) {
      selector.setAttribute('role', 'tablist');

      var select = function (index, focus, updateHash) {
        tabs.forEach(function (tab, i) {
          var on = i === index;
          tab.setAttribute('aria-selected', on ? 'true' : 'false');
          tab.tabIndex = on ? 0 : -1;
          panels[i].hidden = !on;
        });
        if (focus) tabs[index].focus();
        if (updateHash && history.replaceState) {
          history.replaceState(null, '', '#' + panels[index].id);
        }
      };

      tabs.forEach(function (tab, i) {
        tab.addEventListener('click', function () { select(i, false, true); });
        tab.addEventListener('keydown', function (event) {
          var next = null;
          if (event.key === 'ArrowRight' || event.key === 'ArrowDown') next = (i + 1) % tabs.length;
          else if (event.key === 'ArrowLeft' || event.key === 'ArrowUp') next = (i - 1 + tabs.length) % tabs.length;
          else if (event.key === 'Home') next = 0;
          else if (event.key === 'End') next = tabs.length - 1;
          if (next === null) return;
          event.preventDefault();
          select(next, true, true);
        });
      });

      var fromHash = function () {
        var id = location.hash.slice(1);
        var index = panels.findIndex(function (panel) { return panel.id === id; });
        return index < 0 ? 0 : index;
      };

      select(fromHash(), false, false);
      window.addEventListener('hashchange', function () {
        var index = fromHash();
        if (panels[index].hidden) select(index, false, false);
      });

      // In-page links elsewhere on the page ("Rust setup") must open the panel.
      document.addEventListener('click', function (event) {
        var link = event.target.closest && event.target.closest('a[href^="#setup-"]');
        if (!link || selector.contains(link)) return;
        var index = panels.findIndex(function (panel) { return '#' + panel.id === link.getAttribute('href'); });
        if (index >= 0) select(index, false, false);
      });
    }
  }

  // Optional videos: the file may be absent in a clean build. When the last
  // source fails, the poster stays as a plain image and no dead play control
  // is shown. When it loads and motion is welcome, it plays muted in view.
  var reduceMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  Array.prototype.forEach.call(document.querySelectorAll('video'), function (video) {
    var sources = video.querySelectorAll('source');
    var last = sources[sources.length - 1];
    var replaced = false;

    function fallback() {
      if (replaced) return;
      replaced = true;
      var img = document.createElement('img');
      img.src = video.getAttribute('poster') || '';
      img.alt = video.getAttribute('data-alt') || '';
      video.parentNode.replaceChild(img, video);
    }

    if (!last) { fallback(); return; }
    last.addEventListener('error', fallback);
    video.addEventListener('error', fallback);
    // The source may have failed before this script ran (deferred load).
    if (video.error || video.networkState === HTMLMediaElement.NETWORK_NO_SOURCE) fallback();

    if (reduceMotion || !('IntersectionObserver' in window)) return;
    var observer = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (replaced) return;
        if (entry.isIntersecting) {
          var p = video.play();
          if (p && p.catch) p.catch(function () { /* user gesture required; controls remain */ });
        } else {
          video.pause();
        }
      });
    }, { threshold: 0.4 });
    observer.observe(video);
  });
})();
