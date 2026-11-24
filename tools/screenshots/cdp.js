// Minimal headless-Chrome driver over --remote-debugging-pipe (no npm packages needed).
const { spawn } = require('child_process'), fs = require('fs');
class Page {
  constructor(opts = {}) { this.w = opts.width || 1440; this.h = opts.height || 900; this.scale = opts.scale || 1; this.id = 0; this.pending = new Map(); this.handlers = []; this.buf = ''; }
  async launch() {
    this.proc = spawn('/usr/bin/google-chrome', ['--headless=new', '--remote-debugging-pipe', '--no-sandbox', '--disable-gpu', '--hide-scrollbars', '--mute-audio', `--window-size=${this.w},${this.h}`, '--user-data-dir=/tmp/cdp-profile-' + process.pid, 'about:blank'], { stdio: ['ignore', 'ignore', 'ignore', 'pipe', 'pipe'] });
    this.out = this.proc.stdio[3]; this.inp = this.proc.stdio[4];
    this.inp.on('data', (d) => { this.buf += d.toString(); let i; while ((i = this.buf.indexOf('\0')) >= 0) { const m = JSON.parse(this.buf.slice(0, i)); this.buf = this.buf.slice(i + 1); this.onMsg(m); } });
    const { targetInfos } = await this.send('Target.getTargets');
    const t = targetInfos.find((x) => x.type === 'page');
    const { sessionId } = await this.send('Target.attachToTarget', { targetId: t.targetId, flatten: true });
    this.session = sessionId;
    await this.cmd('Page.enable'); await this.cmd('Runtime.enable');
    await this.cmd('Emulation.setDeviceMetricsOverride', { width: this.w, height: this.h, deviceScaleFactor: this.scale, mobile: false });
    await this.cmd('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: 'dark' }] });
  }
  onMsg(m) { if (m.id && this.pending.has(m.id)) { const { res, rej } = this.pending.get(m.id); this.pending.delete(m.id); m.error ? rej(new Error(JSON.stringify(m.error))) : res(m.result); } }
  send(method, params = {}, session) { const id = ++this.id; const msg = { id, method, params }; if (session) msg.sessionId = session; return new Promise((res, rej) => { this.pending.set(id, { res, rej }); this.out.write(JSON.stringify(msg) + '\0'); }); }
  cmd(method, params) { return this.send(method, params, this.session); }
  async goto(url) { await this.cmd('Page.navigate', { url }); await this.sleep(500); }
  async eval(expr) { const r = await this.cmd('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true }); if (r.exceptionDetails) throw new Error(JSON.stringify(r.exceptionDetails)); return r.result.value; }
  sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }
  async waitFor(expr, ms = 30000) { const t = Date.now(); while (Date.now() - t < ms) { try { if (await this.eval(expr)) return true; } catch (e) {} await this.sleep(250); } throw new Error('timeout waiting for ' + expr); }
  // center of the first element whose selector matches (and optionally whose text contains `text`)
  async where(sel, text) { return this.eval(`(()=>{const els=[...document.querySelectorAll(${JSON.stringify(sel)})].filter(e=>!${JSON.stringify(text || '')}||e.textContent.includes(${JSON.stringify(text || '')}));const e=els[0];if(!e)return null;e.scrollIntoView({block:'center'});const r=e.getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2,w:r.width,h:r.height,x0:r.x,y0:r.y}})()`); }
  async mouse(type, x, y, extra = {}) { await this.cmd('Input.dispatchMouseEvent', { type, x, y, button: 'left', buttons: type === 'mouseReleased' ? 0 : 1, clickCount: 1, ...extra }); }
  async clickAt(x, y) { await this.cmd('Input.dispatchMouseEvent', { type: 'mouseMoved', x, y }); await this.mouse('mousePressed', x, y); await this.mouse('mouseReleased', x, y); }
  async click(sel, text) { const p = await this.where(sel, text); if (!p) throw new Error('not found: ' + sel + ' ' + (text || '')); await this.clickAt(p.x, p.y); return p; }
  async drag(x1, y1, x2, y2, steps = 12) { await this.cmd('Input.dispatchMouseEvent', { type: 'mouseMoved', x: x1, y: y1 }); await this.mouse('mousePressed', x1, y1); for (let i = 1; i <= steps; i++) { await this.cmd('Input.dispatchMouseEvent', { type: 'mouseMoved', x: x1 + ((x2 - x1) * i) / steps, y: y1 + ((y2 - y1) * i) / steps, button: 'left', buttons: 1 }); await this.sleep(15); } await this.mouse('mouseReleased', x2, y2); }
  async type(text) { for (const ch of text) { await this.cmd('Input.dispatchKeyEvent', { type: 'keyDown', text: ch, key: ch }); await this.cmd('Input.dispatchKeyEvent', { type: 'keyUp', key: ch }); } }
  async key(key, code) { await this.cmd('Input.dispatchKeyEvent', { type: 'keyDown', key, code: code || key, windowsVirtualKeyCode: { Enter: 13, Tab: 9, Escape: 27 }[key] || 0 }); await this.cmd('Input.dispatchKeyEvent', { type: 'keyUp', key, code: code || key }); }
  async shot(file, clip) { const r = await this.cmd('Page.captureScreenshot', { format: 'png', ...(clip ? { clip: { ...clip, scale: 1 } } : {}) }); fs.writeFileSync(file, Buffer.from(r.data, 'base64')); return file; }
  async close() { try { await this.send('Browser.close'); } catch (e) {} this.proc.kill(); }
}
module.exports = { Page };
