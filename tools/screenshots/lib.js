const { Page } = require('./cdp.js');
const SP = process.env.SP || __dirname, D = process.env.D || SP + '/work/src/robot_demo', OUT = process.env.OUT || SP + '/out';
async function open(file, w = 1200, h = 640, readyText = '') {
  const p = new Page({ width: w, height: h }); await p.launch();
  await p.goto('http://127.0.0.1:8790/open?file=' + encodeURIComponent(file));
  await p.waitFor(`document.querySelector('.rt-toolbar-actions, header') !== null && (${JSON.stringify(readyText)}===''||document.body.innerText.includes(${JSON.stringify(readyText)}))`, 90000);
  await p.sleep(2500); return p;
}
async function fit(p) { await p.click('button[title="Fit view"]'); await p.sleep(600); }
module.exports = { open, fit, D, OUT, SP, Page };
const fs = require('fs'), cp = require('child_process');
class Frames {
  constructor(page, name) { this.p = page; this.dir = require('os').tmpdir() + '/frames-' + name; fs.rmSync(this.dir, { recursive: true, force: true }); fs.mkdirSync(this.dir); this.n = 0; this.delays = []; }
  async snap(delay = 12) { await this.p.shot(`${this.dir}/f${String(this.n++).padStart(3, '0')}.png`); this.delays.push(delay); }
  build(out, width = 960) {
    const args = [];
    for (let i = 0; i < this.n; i++) args.push('-delay', String(this.delays[i]), `${this.dir}/f${String(i).padStart(3, '0')}.png`);
    cp.execFileSync('convert', [...args, '-resize', width + 'x', '-colors', '96', '-layers', 'Optimize', '-loop', '0', out]);
    return out;
  }
}
async function portCenter(p, nodeName, portText) {
  return p.eval(`(()=>{const n=[...document.querySelectorAll('.ros-node')].find(n=>((n.querySelector('.ros-node-title')||{}).lastChild||{}).textContent==${JSON.stringify(nodeName)});if(!n)return null;const pt=[...n.querySelectorAll('.ros-port')].find(x=>x.textContent.includes(${JSON.stringify(portText)}));if(!pt)return null;const r=(pt.querySelector('.ros-port-box')||pt).getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2}})()`);
}
module.exports.Frames = Frames; module.exports.portCenter = portCenter;
