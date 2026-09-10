/* UI layer. Owns the DOM and the local mirror of the board state. */

(() => {

  const $ = id => document.getElementById(id);

  const el = {
    dot:       $('dot'),
    status:    $('status'),
    conn:      $('btn-conn'),
    refresh:   $('btn-refresh'),
    theme:     $('btn-theme'),
    info:      $('btn-info'),
    wipe:      $('btn-wipe'),
    list:      $('list'),
    empty:     $('empty'),
    emptyTxt:  $('empty-txt'),
    count:     $('count'),
    activeNm:  $('active-nm'),
    fire:      $('btn-fire'),
    toast:     $('toast'),
    listenBar: $('listen-bar'),
    listenSec: $('listen-time'),
    listenX:   $('listen-cancel'),
    capInput:  $('cap-input'),
    capCount:  $('cap-count'),
    capTtl:    $('cap-ttl'),
    capDup:    $('cap-dup'),
    capMeta:   $('cap-meta'),
    capActions:$('cap-actions'),
    renInput:  $('ren-input'),
    renCount:  $('ren-count'),
    renSave:   $('ren-save'),
    cfmTtl:    $('cfm-ttl'),
    cfmBody:   $('cfm-body'),
    cfmOk:     $('cfm-ok'),
  };

  let commands   = new Map();
  let activeId   = -1;
  let capture    = null;
  let renameId   = -1;
  let onConfirm  = null;
  let loading    = false;
  let listTimer  = null;
  let toastTimer = null;
  let listenTimer = null;

  /* theme ---------------------------------------------------- */

  const savedTheme = localStorage.getItem('npg-theme');
  if (savedTheme) document.documentElement.dataset.theme = savedTheme;

  el.theme.addEventListener('click', () => {
    const isDark = savedThemeIsDark();
    const next = isDark ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    localStorage.setItem('npg-theme', next);
  });

  function savedThemeIsDark() {
    const set = document.documentElement.dataset.theme;
    if (set) return set === 'dark';
    return matchMedia('(prefers-color-scheme: dark)').matches;
  }

  /* helpers -------------------------------------------------- */

  function toast(message, isError = false) {
    el.toast.textContent = message;
    el.toast.classList.toggle('err', isError);
    el.toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.toast.classList.remove('show'), 2600);
  }

  function setStatus(text) {
    el.status.textContent = text;
  }

  function openModal(id) {
    $(id).classList.add('show');
  }

  function closeModal(id) {
    $(id).classList.remove('show');
  }

  function icon(paths) {
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    svg.setAttribute('viewBox', '0 0 16 16');
    svg.setAttribute('aria-hidden', 'true');
    paths.forEach(d => {
      const p = document.createElementNS('http://www.w3.org/2000/svg', 'path');
      p.setAttribute('d', d);
      svg.appendChild(p);
    });
    return svg;
  }

  const PENCIL = ['M11 2.5l2.5 2.5L6 12.5 3 13l.5-3z'];
  const TRASH  = ['M2.5 4h11', 'M5.5 4V2.5h5V4', 'M4 4l.6 9h6.8L12 4', 'M6.5 6.5v4M9.5 6.5v4'];

  /* rendering ------------------------------------------------ */

  function render() {
    if (loading) return;

    const ids = [...commands.keys()].sort((a, b) => a - b);
    el.count.textContent = ids.length;
    el.list.replaceChildren();

    if (ids.length === 0) {
      el.empty.classList.add('show');
      el.emptyTxt.textContent = BLE.isConnected()
        ? 'No commands saved yet. Hold the user button on the board to learn one.'
        : 'Connect to your NPG board to load commands.';
    } else {
      el.empty.classList.remove('show');
      ids.forEach(id => el.list.appendChild(buildRow(id)));
    }

    const active = commands.get(activeId);
    el.activeNm.textContent = active ? active.name : 'None';
    el.activeNm.classList.toggle('none', !active);
    el.fire.disabled = !BLE.isConnected() || !active;
  }

  function buildRow(id) {
    const cmd = commands.get(id);

    const li = document.createElement('li');
    li.className = 'row' + (id === activeId ? ' active' : '');
    li.tabIndex = 0;

    const radio = document.createElement('span');
    radio.className = 'radio';

    const name = document.createElement('span');
    name.className = 'row-nm';
    name.textContent = cmd.name;

    const actions = document.createElement('span');
    actions.className = 'row-actions';

    const edit = document.createElement('button');
    edit.className = 'icon-btn icon-btn-sm';
    edit.title = 'Rename';
    edit.setAttribute('aria-label', `Rename ${cmd.name}`);
    edit.appendChild(icon(PENCIL));

    const del = document.createElement('button');
    del.className = 'icon-btn icon-btn-sm danger';
    del.title = 'Delete';
    del.setAttribute('aria-label', `Delete ${cmd.name}`);
    del.appendChild(icon(TRASH));

    edit.addEventListener('click', e => { e.stopPropagation(); openRename(id); });
    del.addEventListener('click',  e => { e.stopPropagation(); confirmDelete(id); });

    const activate = () => setActive(id);
    li.addEventListener('click', activate);
    li.addEventListener('keydown', e => {
      if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); activate(); }
    });

    actions.append(edit, del);
    li.append(radio, name, actions);
    return li;
  }

  function setConnectedUI(connected, deviceName) {
    el.dot.classList.toggle('live', connected);
    el.conn.textContent = connected ? 'Disconnect' : 'Connect';
    el.refresh.disabled = !connected;
    el.wipe.disabled = !connected;
    if (connected) setStatus(deviceName);
    el.fire.disabled = !connected || !commands.has(activeId);
  }

  /* actions -------------------------------------------------- */

  async function guard(fn, errorText) {
    try {
      await fn();
    } catch (err) {
      toast(errorText || err.message, true);
    }
  }

  function setActive(id) {
    if (id === activeId) return;
    activeId = id;
    render();
    guard(() => BLE.setActive(id));
  }

  function confirmDelete(id) {
    const cmd = commands.get(id);
    if (!cmd) return;
    el.cfmTtl.textContent = 'Delete command';
    el.cfmBody.textContent = `"${cmd.name}" will be removed from the board. This cannot be undone.`;
    el.cfmOk.textContent = 'Delete';
    onConfirm = () => guard(() => BLE.remove(id));
    openModal('ov-confirm');
  }

  function confirmWipe() {
    el.cfmTtl.textContent = 'Erase all commands';
    el.cfmBody.textContent =
      `All ${commands.size} saved command${commands.size === 1 ? '' : 's'} and their IR signals ` +
      'will be erased from the board. This cannot be undone.';
    el.cfmOk.textContent = 'Erase all';
    onConfirm = () => guard(() => BLE.wipe());
    openModal('ov-confirm');
  }

  function openRename(id) {
    renameId = id;
    el.renInput.value = commands.get(id)?.name || '';
    el.renCount.textContent = el.renInput.value.length;
    el.renInput.classList.remove('invalid');
    openModal('ov-rename');
    setTimeout(() => el.renInput.focus(), 50);
  }

  function submitRename() {
    const name = el.renInput.value.trim();
    if (!name) {
      el.renInput.classList.add('invalid');
      el.renInput.focus();
      return;
    }
    closeModal('ov-rename');
    guard(() => BLE.rename(renameId, name));
  }

  /* listening ------------------------------------------------ */

  function showListening(seconds) {
    // the overlay must be visible first, a hidden element has no layout
    // to reflow and the bar would jump straight to zero
    openModal('ov-listening');

    const bar = el.listenBar;
    bar.style.transition = 'none';
    bar.style.width = '100%';
    void bar.offsetWidth;                    // commit the reset before animating
    bar.style.transition = `width ${seconds}s linear`;
    bar.style.width = '0%';

    let left = seconds;
    el.listenSec.textContent = `${left}s`;
    clearInterval(listenTimer);
    listenTimer = setInterval(() => {
      left -= 1;
      el.listenSec.textContent = `${Math.max(left, 0)}s`;
      if (left <= 0) clearInterval(listenTimer);
    }, 1000);
  }

  function hideListening() {
    clearInterval(listenTimer);
    listenTimer = null;
    closeModal('ov-listening');
  }

  function cancelListening() {
    hideListening();
    guard(() => BLE.cancelRec());
  }

  // the board holds the capture in RAM, so every way of dismissing this
  // dialog has to tell it to let go or the button stays stuck
  function discardCapture() {
    closeModal('ov-capture');
    capture = null;
    guard(() => BLE.discard());
  }

  /* capture flow --------------------------------------------- */

  function showCapture(data) {
    capture = data;
    const dup = data.isDuplicate ? commands.get(data.duplicateId) : null;
    const dupName = dup ? dup.name : `slot ${data.duplicateId}`;

    el.capTtl.textContent = data.isDuplicate ? 'Signal already saved' : 'IR signal captured';

    el.capDup.replaceChildren();
    if (data.isDuplicate) {
      const notice = document.createElement('div');
      notice.className = 'notice';
      notice.appendChild(icon(['M8 1.5L15 14H1z', 'M8 6.5v3', 'M8 11.5h.01']));
      const text = document.createElement('span');
      text.textContent = `This matches "${dupName}". Save it as a new command or replace the existing one.`;
      notice.appendChild(text);
      el.capDup.appendChild(notice);
    }

    el.capMeta.replaceChildren();
    addMeta('Protocol', BLE.protocolName(data.protocol));
    addMeta('Bits', String(data.bits));
    if (data.protocol !== 0) addMeta('Value', `0x${data.value}`);

    el.capInput.value = '';
    el.capCount.textContent = '0';
    el.capInput.classList.remove('invalid');

    el.capActions.replaceChildren();
    addCaptureAction('Discard', 'btn', discardCapture);
    if (data.isDuplicate) {
      addCaptureAction('Replace', 'btn btn-danger', () => {
        const name = el.capInput.value.trim() || dupName;
        closeModal('ov-capture');
        guard(() => BLE.saveOver(data.duplicateId, name));
      });
    }
    addCaptureAction(data.isDuplicate ? 'Save as new' : 'Save', 'btn btn-primary', () => {
      const name = el.capInput.value.trim();
      if (!name) {
        el.capInput.classList.add('invalid');
        el.capInput.focus();
        return;
      }
      closeModal('ov-capture');
      guard(() => BLE.saveNew(name));
    });

    openModal('ov-capture');
    setTimeout(() => el.capInput.focus(), 50);
  }

  function addMeta(term, value) {
    const dt = document.createElement('dt');
    dt.textContent = term;
    const dd = document.createElement('dd');
    dd.textContent = value;
    el.capMeta.append(dt, dd);
  }

  function addCaptureAction(label, className, handler) {
    const btn = document.createElement('button');
    btn.className = className;
    btn.textContent = label;
    btn.addEventListener('click', handler);
    el.capActions.appendChild(btn);
  }

  /* board events --------------------------------------------- */

  BLE.on('connect', name => {
    commands.clear();
    activeId = -1;
    setConnectedUI(true, name);
    render();
    requestList();
  });

  BLE.on('disconnect', () => {
    commands.clear();
    activeId = -1;
    loading = false;
    clearTimeout(listTimer);
    hideListening();
    // the board drops its pending capture on disconnect, so close the
    // dialog rather than leaving a Save button that cannot work
    closeModal('ov-capture');
    capture = null;
    setConnectedUI(false);
    setStatus('Not connected');
    render();
  });

  BLE.on('listEntry', entry => {
    commands.set(entry.id, { name: entry.name });
    if (entry.active) activeId = entry.id;
    loading = true;
    // the board may never send list-end if a packet is dropped
    clearTimeout(listTimer);
    listTimer = setTimeout(finishList, 700);
  });

  BLE.on('listEnd', () => {
    clearTimeout(listTimer);
    finishList();
  });

  BLE.on('saved', cmd => {
    commands.set(cmd.id, { name: cmd.name });
    if (activeId < 0) activeId = cmd.id;
    loading = false;
    render();
    toast(`Saved "${cmd.name}"`);
  });

  BLE.on('deleted', id => {
    commands.delete(id);
    if (activeId === id) activeId = -1;
    loading = false;
    render();
    toast('Command deleted');
  });

  BLE.on('active', id => {
    activeId = id;
    loading = false;
    render();
  });

  BLE.on('wiped', () => {
    commands.clear();
    activeId = -1;
    loading = false;
    render();
    toast('All commands erased');
  });

  BLE.on('fired', () => {
    const cmd = commands.get(activeId);
    toast(cmd ? `Sent "${cmd.name}"` : 'Sent');
  });

  BLE.on('failed', () => toast('The board could not complete that', true));

  BLE.on('listening', showListening);

  BLE.on('listenEnd', reason => {
    hideListening();
    if (reason === 0) toast('No signal received', true);
    else if (reason === 1) toast('Recording cancelled');
  });

  BLE.on('capture', data => {
    hideListening();
    showCapture(data);
  });

  function requestList() {
    loading = true;
    setStatus('Loading commands');
    guard(() => BLE.getList());
    clearTimeout(listTimer);
    listTimer = setTimeout(finishList, 3000);
  }

  function finishList() {
    loading = false;
    render();
    const n = commands.size;
    setStatus(`${BLE.DEVICE_NAME} · ${n} command${n === 1 ? '' : 's'}`);
  }

  /* wiring --------------------------------------------------- */

  el.conn.addEventListener('click', async () => {
    if (BLE.isConnected()) {
      BLE.disconnect();
      return;
    }
    setStatus('Scanning');
    try {
      await BLE.connect();
    } catch (err) {
      setStatus('Not connected');
      if (err.name !== 'NotFoundError') toast(err.message, true);
    }
  });

  el.refresh.addEventListener('click', () => {
    commands.clear();
    activeId = -1;
    requestList();
  });

  el.fire.addEventListener('click', () => {
    if (activeId < 0) return;
    guard(() => BLE.fire(activeId));
  });

  el.info.addEventListener('click', () => openModal('ov-info'));
  el.wipe.addEventListener('click', () => {
    closeModal('ov-info');
    confirmWipe();
  });

  el.renSave.addEventListener('click', submitRename);
  el.cfmOk.addEventListener('click', () => {
    closeModal('ov-confirm');
    onConfirm?.();
    onConfirm = null;
  });

  bindCounter(el.capInput, el.capCount);
  bindCounter(el.renInput, el.renCount);

  function bindCounter(input, output) {
    input.addEventListener('input', () => {
      output.textContent = input.value.length;
      input.classList.remove('invalid');
    });
    input.addEventListener('keydown', e => {
      if (e.key !== 'Enter') return;
      e.preventDefault();
      if (input === el.renInput) submitRename();
      else el.capActions.lastElementChild?.click();
    });
  }

  // two overlays mirror state the board is holding, so closing them has to
  // tell the board. every dismissal route goes through here.
  function dismiss(id) {
    if (id === 'ov-listening') cancelListening();
    else if (id === 'ov-capture') discardCapture();
    else closeModal(id);
  }

  document.querySelectorAll('[data-close]').forEach(btn => {
    btn.addEventListener('click', () => dismiss(btn.dataset.close));
  });

  el.listenX.addEventListener('click', cancelListening);

  document.querySelectorAll('.overlay').forEach(overlay => {
    overlay.addEventListener('mousedown', e => {
      if (e.target === overlay) dismiss(overlay.id);
    });
  });

  document.addEventListener('keydown', e => {
    if (e.key !== 'Escape') return;
    const open = document.querySelector('.overlay.show');
    if (open) dismiss(open.id);
  });

  /* start ---------------------------------------------------- */

  if (!BLE.isSupported()) {
    el.conn.disabled = true;
    setStatus('Web Bluetooth not supported');
    const notice = document.createElement('div');
    notice.className = 'notice';
    notice.appendChild(icon(['M8 1.5L15 14H1z', 'M8 6.5v3', 'M8 11.5h.01']));
    const text = document.createElement('span');
    text.textContent = 'This browser cannot talk to Bluetooth devices. Open this page in Chrome or Edge.';
    notice.appendChild(text);
    document.querySelector('.main').prepend(notice);
  }

  render();

})();
