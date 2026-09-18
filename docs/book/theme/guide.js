// franka-rs guide. Progressive enhancement over mdBook's sidebar: each part title from
// SUMMARY.md becomes a button that folds the chapters under it. The group holding the
// current page is always open; the others start closed and remember the reader's choice.
// Without this file every group is open, and nothing else depends on it.
(function () {
  'use strict';

  var KEY = 'franka-guide-open-parts';

  function load() {
    try {
      var value = JSON.parse(localStorage.getItem(KEY) || '{}');
      return value && typeof value === 'object' ? value : {};
    } catch (e) {
      return {};
    }
  }

  function save(state) {
    try { localStorage.setItem(KEY, JSON.stringify(state)); } catch (e) { /* private mode */ }
  }

  function init() {
    var list = document.querySelector('#mdbook-sidebar ol.chapter, #sidebar ol.chapter');
    if (!list || list.querySelector('.guide-part-toggle')) return false;

    // Resolve from the introduction so this also works on nested pages and local previews.
    var introduction = list.querySelector('a[href$="introduction.html"]');
    if (introduction && !list.querySelector('.guide-home')) {
      var homeItem = document.createElement('li');
      homeItem.className = 'chapter-item';
      var homeLink = document.createElement('a');
      homeLink.className = 'guide-home';
      homeLink.href = new URL('index.html', introduction.href).href;
      homeLink.textContent = 'Project home';
      homeItem.appendChild(homeLink);
      list.insertBefore(homeItem, list.firstChild);
    }

    var groups = [];
    var current = null;
    Array.prototype.forEach.call(list.children, function (li) {
      if (li.classList.contains('part-title')) {
        current = { title: li, items: [] };
        groups.push(current);
      } else if (li.classList.contains('spacer')) {
        current = null;
      } else if (current) {
        current.items.push(li);
      }
    });
    if (!groups.length) return true;

    var state = load();
    groups.forEach(function (group) {
      var name = group.title.textContent.trim();
      var holdsActive = group.items.some(function (li) { return li.querySelector('a.active'); });
      var open = holdsActive || state[name] === true;

      var button = document.createElement('button');
      button.type = 'button';
      button.className = 'guide-part-toggle';
      button.textContent = name;
      group.title.textContent = '';
      group.title.appendChild(button);

      function set(isOpen) {
        button.setAttribute('aria-expanded', isOpen ? 'true' : 'false');
        group.items.forEach(function (li) { li.classList.toggle('guide-hidden', !isOpen); });
      }

      set(open);
      button.addEventListener('click', function () {
        var next = button.getAttribute('aria-expanded') !== 'true';
        set(next);
        state[name] = next;
        save(state);
      });
    });
    return true;
  }

  // The sidebar is filled by mdBook's own script; in older builds it is static HTML.
  if (!init()) {
    document.addEventListener('DOMContentLoaded', init);
    window.addEventListener('load', init);
  }
})();
