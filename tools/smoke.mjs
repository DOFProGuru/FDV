/**
 * An end-to-end smoke test in a real browser.
 *
 * The unit tests cover the numbers; they cannot say whether the page actually assembles itself,
 * whether WebGL came up, or whether a NaN escaped into the DOM. This drives a headless Chrome over
 * the DevTools protocol - no browser-automation dependency, Node has had a WebSocket client since
 * v22 - loads a flight, and reports what the page ended up showing and what it complained about.
 *
 *   node tools/smoke.mjs [--url http://localhost:4173/] [--flight f17-nominal] [--wait 8000]
 *
 * Start `npx vite preview --port 4173` (or `vite dev`) first. Exits non-zero if the page threw, if
 * the status line reported an error, or if the panels are still empty.
 */
import { spawn } from 'node:child_process';
import { existsSync, writeFileSync } from 'node:fs';

const CHROME =
  process.env.CHROME ??
  [
    '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
    '/Applications/Chromium.app/Contents/MacOS/Chromium',
    '/usr/bin/google-chrome',
    '/usr/bin/chromium',
  ].find((p) => existsSync(p));

const arg = (name, dflt) => {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 ? process.argv[i + 1] : dflt;
};

const base = arg('url', 'http://localhost:4173/');
const flight = arg('flight', 'f17-nominal');
const waitMs = Number(arg('wait', 9000));
const shot = arg('shot', '');
const profile = `/tmp/flight-smoke-profile-${process.pid}`;

if (!CHROME) {
  console.error('no Chrome found; set CHROME=/path/to/chrome');
  process.exit(2);
}

const chrome = spawn(
  CHROME,
  [
    '--headless=new',
    '--disable-gpu',
    '--use-angle=swiftshader',
    '--enable-unsafe-swiftshader',
    '--no-first-run',
    '--no-default-browser-check',
    '--disable-features=Translate',
    `--user-data-dir=${profile}`,
    '--remote-debugging-port=0',
    '--remote-allow-origins=*',
    '--window-size=1600,1000',
    `${base}#${flight}`,
  ],
  { stdio: ['ignore', 'ignore', 'pipe'] },
);

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

let errText = '';
chrome.stderr.on('data', (d) => (errText += d.toString()));

/** Chrome prints its DevTools websocket URL on stderr when started with --remote-debugging-port=0. */
const endpoint = async () => {
  const started = Date.now();
  for (;;) {
    const m = /DevTools listening on (ws:\/\/\S+)/.exec(errText);
    if (m) return m[1];
    if (Date.now() - started > 20000) throw new Error('Chrome never announced its DevTools endpoint');
    await sleep(100);
  }
};

/** The HTTP side of that endpoint, from which the page targets are listed. */
const httpBase = async () => {
  const ws = await endpoint();
  const http = ws.replace(/^ws:/, 'http:').replace(/\/devtools\/browser.*$/, '');
  const started = Date.now();
  for (;;) {
    const list = await fetch(`${http}/json/list`).then((r) => r.json()).catch(() => []);
    const page = list.find((t) => t.type === 'page' && t.webSocketDebuggerUrl);
    if (page) return { http, target: page };
    if (Date.now() - started > 20000) throw new Error(`no page target at ${http}/json/list`);
    await sleep(100);
  }
};

const fail = (msg) => {
  console.error(`\nFAIL  ${String(msg).split('\n').join('\n      ')}`);
  if (errText) console.error(errText.split('\n').slice(-6).join('\n'));
  chrome.kill('SIGKILL');
  process.exit(1);
};

const run = async () => {
  const { target } = await httpBase();

  const sock = new WebSocket(target.webSocketDebuggerUrl);
  const problems = [];
  let id = 0;
  const pending = new Map();
  const send = (method, params) =>
    new Promise((resolve, reject) => {
      const n = ++id;
      pending.set(n, { resolve, reject });
      sock.send(JSON.stringify({ id: n, method, params: params ?? {} }));
    });

  await new Promise((res, rej) => {
    sock.onopen = res;
    sock.onerror = () => rej(new Error('DevTools socket failed'));
  });
  sock.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.id && pending.has(m.id)) {
      const p = pending.get(m.id);
      pending.delete(m.id);
      m.error ? p.reject(new Error(`${m.error.message}`)) : p.resolve(m.result);
      return;
    }
    if (m.method === 'Runtime.exceptionThrown') {
      const d = m.params.exceptionDetails;
      problems.push(`uncaught: ${d.exception?.description ?? d.text}`);
    } else if (m.method === 'Runtime.consoleAPICalled' && (m.params.type === 'error' || m.params.type === 'warning')) {
      problems.push(`console.${m.params.type}: ${m.params.args.map((a) => a.value ?? a.description ?? a.type).join(' ')}`);
    } else if (m.method === 'Log.entryAdded' && m.params.entry.level === 'error') {
      problems.push(`log: ${m.params.entry.text}`);
    }
  };

  await send('Runtime.enable');
  await send('Log.enable');
  await sleep(waitMs);

  const probe = await send('Runtime.evaluate', {
    expression: `(${(() => {
      const q = (s) => Array.from(document.querySelectorAll(s));
      const txt = (s) => q(s).map((x) => (x.textContent ?? '').replace(/\s+/g, ' ').trim());
      return {
        hidden: document.querySelector('#app')?.hasAttribute('hidden') ?? true,
        status: (document.querySelector('#status')?.textContent ?? '').replace(/\s+/g, ' ').trim(),
        statusClass: document.querySelector('#status')?.className ?? '',
        stats: txt('.stat'),
        canvases: q('canvas').length,
        // The stylesheet must be in. Without it every panel is unstyled, the canvases are sized from
        // a host as tall as their own content, and the charts silently draw into a canvas millions of
        // pixels high - which the browser declines to render at all.
        layout: [...document.querySelectorAll('.chart')].map((e) => {
          const c = getComputedStyle(e);
          return { h: e.offsetHeight, pos: c.position };
        }),
        replay: (() => {
          const c = document.querySelector('#replay canvas');
          return c ? { css: [c.offsetWidth, c.offsetHeight], buf: [c.width, c.height] } : null;
        })(),
        // How much of each chart's canvas carries drawn ink. Reading pixels from a
        // device-pixel-ratio-scaled canvas at full size runs the headless renderer out of memory, so
        // each chart is scaled down into a 64x40 thumbnail first: 2560 cells is plenty to tell a
        // drawn series from an empty panel.
        chartInk: [...document.querySelectorAll('.chart')].map((p) => {
          const src = p.querySelector('canvas');
          if (!src || !src.width) return 0;
          const t = document.createElement('canvas');
          t.width = 64;
          t.height = 40;
          const g = t.getContext('2d');
          if (!g) return 0;
          g.drawImage(src, 0, 0, 64, 40);
          const d = g.getImageData(0, 0, 64, 40).data;
          let lit = 0;
          for (let i = 0; i < d.length; i += 4) if (d[i + 3] > 8) lit++;
          return lit;
        }),
        webgl: (() => {
          const c = document.createElement('canvas');
          return !!(c.getContext('webgl2') || c.getContext('webgl'));
        })(),
        readout: (document.querySelector('#replay-readout')?.textContent ?? '').replace(/\s+/g, ' ').trim(),
        events: txt('#events li'),
        diagTerms: txt('#diag dt'),
        diagValues: txt('#diag dd'),
        warnings: txt('#warnings li'),
        rates: (document.querySelector('#rate')?.selectedOptions?.[0]?.textContent ?? '').trim(),
        scrubMax: document.querySelector('#scrub')?.max ?? '',
        nan: document.body.innerHTML.includes('NaN'),
      };
    }).toString()})()`,
    returnByValue: true,
  }).catch((e) => fail(`evaluate failed: ${e.message}`));

  if (probe.exceptionDetails) {
    fail(`the probe itself failed: ${probe.exceptionDetails.exception?.description ?? probe.exceptionDetails.text}`);
  }
  const s = probe.result?.value ?? {};
  const bad = [];
  if (!s.webgl) bad.push('this Chrome has no WebGL, the 3-D view cannot be judged');
  if (s.hidden) bad.push('#app is still hidden: no flight was rendered');
  if (/error/i.test(s.statusClass ?? '')) bad.push(`status reports an error: ${s.status}`);
  if (/work/.test(s.statusClass ?? '')) bad.push(`still working after ${waitMs} ms: ${s.status}`);
  if (s.stats.length !== 7) bad.push(`expected 7 KPI cells, found ${s.stats.length}`);
  if (s.canvases < 8) bad.push(`expected 8 canvases (4 charts x data+overlay), found ${s.canvases}`);
  (s.chartInk ?? []).forEach((ink, i) => {
    if (ink < 40) bad.push(`chart ${i + 1} has drawn almost nothing (${ink} of 2560 cells)`);
  });
  if (!s.layout?.length) bad.push('no chart panels found');
  if (!s.replay || s.replay.css[0] < 100 || s.replay.css[1] < 100 || s.replay.buf[0] < 100)
    bad.push(`the 3-D view has no canvas of a usable size: ${JSON.stringify(s.replay)}`);
  s.layout?.forEach((l, i) => {
    if (l.pos !== 'relative')
      bad.push(`chart ${i + 1} is not styled (position: ${l.pos}) - the stylesheet did not load`);
    if (l.h < 80 || l.h > 1200) bad.push(`chart ${i + 1} is ${l.h}px tall, which is not a chart height`);
  });
  if (s.events.length < 5) bad.push(`expected at least 5 events, found ${s.events.length}`);
  if (s.diagTerms.length < 8) bad.push(`expected at least 8 diagnostic rows, found ${s.diagTerms.length}`);
  if (s.nan) bad.push('the literal text "NaN" reached the DOM');
  if (!/^T[+−-]/.test(s.readout)) bad.push(`the replay readout did not update: "${s.readout}"`);
  if (+s.scrubMax < 20) bad.push(`the scrubber range looks wrong (max ${s.scrubMax})`);
  for (const v of [...s.diagValues, ...s.stats]) if (/undefined|\[object/.test(v)) bad.push(`unrendered value in the DOM: "${v}"`);
  // Chrome itself narrates the software renderer and the missing favicon; those are not the page's
  // doing, and this headless Chrome is the only one available without a GPU.
  const noise = /favicon|swiftshader|software rendering|software webgl|GroupMarkerNotSet|Automatic fallback to software/i;
  const realProblems = problems.filter((p) => !noise.test(p));
  for (const p of realProblems) bad.push(p);

  if (shot) {
    await send('Page.enable');
    const { data } = await send('Page.captureScreenshot', { format: 'png' });
    writeFileSync(shot, Buffer.from(data, 'base64'));
    console.log(`screenshot ${shot}`);
  }

  console.log(`page      ${s.status}`);
  console.log(`KPIs      ${s.stats.join(' | ')}`);
  console.log(`readout   ${s.readout}`);
  console.log(`chart ink ${s.chartInk.join(' / ')} of 2560 cells each`);
  console.log(`layout    ${s.layout.map((l) => `${l.h}px/${l.pos}`).join(' / ')}`);
  console.log(`replay    ${s.replay ? `${s.replay.css.join('x')} css, ${s.replay.buf.join('x')} buffer` : 'no canvas'}`);
  console.log(`events    ${s.events.slice(0, 8).join(' | ')}`);
  console.log(`warnings  ${s.warnings.join(' | ') || '(none)'}`);
  console.log('diagnostics');
  s.diagTerms.forEach((k, i) => console.log(`  ${k.padEnd(22)} ${s.diagValues[i] ?? ''}`));
  if (bad.length) {
    console.log('');
    fail(bad.join('\n      '));
  }
  console.log(`\nPASS  ${flight} rendered in a real browser (${s.canvases} canvases, WebGL ${s.webgl ? 'ok' : 'absent'})`);
  sock.close();
  chrome.kill('SIGKILL');
  process.exit(0);
};

run().catch((e) => fail(e.stack ?? String(e)));
