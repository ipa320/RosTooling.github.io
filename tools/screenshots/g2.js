const { open, fit, D, OUT, Frames } = require('./lib.js');
const fs = require('fs');
(async () => {
  fs.copyFileSync(__dirname + '/ac_system.orig', D + '/ac_system.rossystem');
  const p = await open(D + '/ac_system.rossystem', 1200, 940, 'planner'); await fit(p);
  const f = new Frames(p, 'autoconnect'); await f.snap(70);
  await p.click('button', 'Auto-connect'); await p.waitFor(`document.querySelector('.rt-ac-panel') && document.querySelector('.rt-ac-panel').innerText.length>80`, 40000); await p.sleep(1200); await f.snap(110);
  // choose laser::scan for filter.scan_in (first receiver group)
  const r = await p.where('.rt-ac-cand', 'laser::scan'); await p.clickAt(r.x0 + 14, r.y); await p.sleep(500); await f.snap(110);
  await p.click('button', 'Add 2 connections'); await p.sleep(3000); await f.snap(110);
  await p.waitFor(`!document.querySelector('.rt-ac-panel') || document.querySelector('.rt-ac-panel').innerText.includes('0 open') || true`); await f.snap(150);
  f.build(OUT + '/studio-autoconnect.gif', 900); await p.shot(OUT + '/studio-autoconnect-after.png'); await p.close(); console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
