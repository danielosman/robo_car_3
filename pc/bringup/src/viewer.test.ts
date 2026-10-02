import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';

// Lightweight DOM/WebSocket smoke test: exercises the actual browser entrypoint
// without serial hardware. Not a substitute for visual browser/PCB verification.
test('viewer renders both streams and sends independent sensor controls', async () => {
  const html = readFileSync(new URL('../public/index.html', import.meta.url), 'utf8');
  const ids = new Set([...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]));
  class Element {
    value = ''; checked = false; disabled = false; max = '15'; width = 160; height = 120;
    textContent = ''; className = ''; hidden = false; title = ''; scrollTop = 0; scrollHeight = 0;
    style: Record<string, string> = {}; children: Element[] = [];
    classList = {add: (name: string) => { this.className += ' ' + name; }};
    onclick?: () => void; onchange?: () => void; oninput?: () => void;
    append(...children: Element[]) { this.children.push(...children); }
    replaceChildren() { this.children = []; }
    getContext() { return {createImageData: (w: number, h: number) => ({data: new Uint8ClampedArray(w*h*4)}), putImageData: () => { draws++; }}; }
  }
  const nodes = new Map([...ids].map(id => [id, new Element()]));
  const node = (id: string) => { assert.ok(nodes.has(id), `missing HTML element ${id}`); return nodes.get(id)!; };
  Object.entries({gamma: '1', metric: 'distance', target: 'closest', validity: 'normal', rotation: '0',
    resolution: '64', hz: '10', order: '1', ranging: '1', integration: '5', sharpener: '5'}).forEach(([id, value]) => { node(id).value = value; });
  let draws = 0, timer = () => {};
  const sent: unknown[] = [];
  let socket: FakeSocket;
  class FakeSocket {
    static OPEN = 1; readyState = 1; binaryType = '';
    onmessage?: (ev: {data: string | ArrayBuffer}) => void;
    constructor(_url: string) { socket = this; }
    send(data: unknown) { sent.push(data); }
  }
  const saved = Object.getOwnPropertyDescriptors(globalThis);
  const mocks = {
    document: {getElementById: node, createElement: () => new Element()},
    location: {host: 'localhost:8080'}, WebSocket: FakeSocket,
    setInterval: (fn: () => void) => { timer = fn; return 1; },
  };
  for (const [name, value] of Object.entries(mocks)) Object.defineProperty(globalThis, name, {configurable: true, writable: true, value});
  try {
    await import('../public/viewer.js');
    assert.equal(node('rotation').value, '90', 'PCB mounting rotation is the startup default');
    const receive = (msg: object) => socket.onmessage!({data: JSON.stringify(msg)});
    receive({type: 'connection', connected: true, port: 'COM_TEST'});
    receive({type: 'log', text: 'MODE 3: H_SUB=2 V_SUB=2 BIN=0 capture=160x120 OK'});
    assert.equal(node('bin').value, '3');
    const camera = new Uint8Array(4 + 160*120); camera.set([160, 0, 120, 0]);
    socket!.onmessage!({data: camera.buffer}); assert.equal(draws, 1);
    const zones = Array.from({length: 64}, () => ({count: 2, ambient: 10, spads: 50,
      targets: [{distance: 100, signal: 30, sigma: 4, reflectance: 20, status: 5},
        {distance: 1000, signal: 10, sigma: 8, reflectance: 10, status: 9}]}));
    receive({type: 'tof', side: 8, hz: 10, sequence: 0, temperature: 25, order: 1, mode: 1, sharpener: 4, zones});
    assert.equal(node('grid').children.length, 64);
    assert.equal(node('grid').children[0].children[0].textContent, '100');
    node('target').value = 'farthest'; node('target').onchange!();
    assert.equal(node('grid').children[0].children[0].textContent, '1000');
    node('bin').value = '2'; node('bin').onchange!();
    node('stopTof').onclick!(); node('applyTof').onclick!();
    assert.deepEqual(sent, [new Uint8Array([0x4d, 2]), 'S', 'T stop\n', 'T 64 10 1 1 5 5\n']);
    node('ranging').value = '3'; node('integration').value = '100'; node('applyTof').onclick!();
    assert.equal(sent.length, 4, 'invalid autonomous timing must not be sent');
    sent.length = 0;
    receive({type: 'cameraSettings', ae: false, lines: 300, maxLines: 560, analog: 2, digital: 100, tone: 3});
    assert.equal(node('tone').value, '3'); assert.equal(node('ae').value, '0');
    assert.equal(node('manual').hidden, false); assert.equal(node('lines').max, '560');
    assert.equal(node('digital').value, '100', 'non-preset digital gain from readback is shown');
    node('tone').value = '1'; node('tone').onchange!();
    node('lines').value = '400'; node('lines').onchange!();
    node('ae').value = '1'; node('ae').onchange!();
    assert.deepEqual(sent, ['H tone 1\n', 'H manual 400 2 100\n', 'H auto\n']);
    assert.equal(node('manual').hidden, true);
    const channel = (adc: number) => ({adc, exponent: 1, counter: 2, crcOk: true, rangeOk: true});
    receive({type: 'opt', sequence: 5, conversionMs: 100, overload: false, valid: true, lux: 123.4,
      channels: [channel(10), channel(20), channel(30), channel(40)],
      xyz: {X: 1, Y: 1, Z: 1}, chromaticity: {x: 0.3127, y: 0.329}});
    assert.equal(node('lux').textContent, '123.4');
    assert.match(node('optXy').textContent, /0\.3127, 0\.3290/);
    assert.match(node('swatch').style.background, /^rgb\(/);
    node('initOpt').onclick!(); assert.equal(sent.at(-1), 'O init\n');
    timer(); assert.match(node('cameraStatus').textContent, /160×120/);
    assert.match(node('tofStatus').textContent, /8×8/);
    assert.match(node('optStatus').textContent, /#5 .* valid/);
  } finally {
    for (const name of Object.keys(mocks)) {
      if (saved[name]) Object.defineProperty(globalThis, name, saved[name]);
      else Reflect.deleteProperty(globalThis, name);
    }
  }
});
