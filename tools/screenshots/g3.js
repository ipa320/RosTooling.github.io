const { open, fit, D, OUT, Frames } = require('./lib.js');
const fs = require('fs');
(async () => {
  fs.writeFileSync(D + '/import_system.rossystem', 'import_system:\n  nodes:\n    "laser":\n      from: "scanner_pkg.laser_driver"\n');
  const p = await open(D + '/import_system.rossystem', 1200, 620, 'laser'); await fit(p);
  const f = new Frames(p, 'import'); await f.snap(70);
  await p.click('#btnAddSystemNode'); await p.sleep(1400); await f.snap(90);
  const s = await p.where('.rt-search-box input'); await p.clickAt(s.x, s.y); await p.type('scan_filter'); await p.sleep(900); await f.snap(90);
  await p.click('aside button', 'Import'); await p.sleep(5000); await f.snap(100); await fit(p); await p.sleep(800); await f.snap(150);
  f.build(OUT + '/studio-import-node.gif', 900); await p.shot(OUT + '/studio-import-node-after.png'); await p.close(); console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
