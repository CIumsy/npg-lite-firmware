/* Lucide icons, inlined.
 *
 * The SVG files in assets/ are the originals from lucide.dev. Their inner
 * markup is copied here verbatim so the icons can inherit colour from CSS,
 * which an <img> cannot do. Keys match the Lucide names, so to add one,
 * download it from lucide.dev and paste its inner markup below. */

const Icons = (() => {
  'use strict';

  const NS = 'http://www.w3.org/2000/svg';

  const SHAPES = {
    'sun':
      '<circle cx="12" cy="12" r="4"/><path d="M12 2v2"/><path d="M12 20v2"/>' +
      '<path d="m4.93 4.93 1.41 1.41"/><path d="m17.66 17.66 1.41 1.41"/>' +
      '<path d="M2 12h2"/><path d="M20 12h2"/><path d="m6.34 17.66-1.41 1.41"/>' +
      '<path d="m19.07 4.93-1.41 1.41"/>',

    'moon':
      '<path d="M20.985 12.486a9 9 0 1 1-9.473-9.472c.405-.022.617.46.402.803a6 6 0 0 0 8.268 8.268c.344-.215.825-.004.803.401"/>',

    'info':
      '<circle cx="12" cy="12" r="10"/><path d="M12 16v-4"/><path d="M12 8h.01"/>',

    'bluetooth':
      '<path d="m7 7 10 10-5 5V2l5 5L7 17"/>',

    'power':
      '<path d="M12 2v10"/><path d="M18.4 6.6a9 9 0 1 1-12.77.04"/>',

    'pencil':
      '<path d="M21.174 6.812a1 1 0 0 0-3.986-3.987L3.842 16.174a2 2 0 0 0-.5.83l-1.321 4.352a.5.5 0 0 0 .623.622l4.353-1.32a2 2 0 0 0 .83-.497z"/>' +
      '<path d="m15 5 4 4"/>',

    'trash':
      '<path d="M10 11v6"/><path d="M14 11v6"/>' +
      '<path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6"/><path d="M3 6h18"/>' +
      '<path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>',

    'x':
      '<path d="M18 6 6 18"/><path d="m6 6 12 12"/>',

    'refresh-cw':
      '<path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"/>' +
      '<path d="M21 3v5h-5"/>' +
      '<path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"/>' +
      '<path d="M8 16H3v5"/>',

    'triangle-alert':
      '<path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3"/>' +
      '<path d="M12 9v4"/><path d="M12 17h.01"/>',

    'radio-tower':
      '<path d="M4.9 16.1C1 12.2 1 5.8 4.9 1.9"/>' +
      '<path d="M7.8 4.7a6.14 6.14 0 0 0-.8 7.5"/><circle cx="12" cy="9" r="2"/>' +
      '<path d="M16.2 4.8c2 2 2.26 5.11.8 7.47"/>' +
      '<path d="M19.1 1.9a9.96 9.96 0 0 1 0 14.1"/><path d="M9.5 18h5"/>' +
      '<path d="m8 22 4-11 4 11"/>'
  };

  // Stroke width, caps and colour come from the stylesheet, so the icons
  // match the text around them at every size.
  function svg(name) {
    const node = document.createElementNS(NS, 'svg');
    node.setAttribute('viewBox', '0 0 24 24');
    node.setAttribute('aria-hidden', 'true');
    node.innerHTML = SHAPES[name] || '';
    return node;
  }

  // Fills in every <span class="ico" data-icon="name"> under root.
  function hydrate(root) {
    (root || document).querySelectorAll('[data-icon]').forEach(slot => {
      slot.replaceChildren(svg(slot.dataset.icon));
    });
  }

  return { svg, hydrate };
})();
