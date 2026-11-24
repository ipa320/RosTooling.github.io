const { open, fit, D, OUT } = require('./lib.js');
(async () => {
  let p = await open(D + '/proc_system.rossystem', 1200, 560, 'scan_det'); await fit(p);
  const t = await p.where('text', 'Process: perception'); await p.clickAt(t.x, t.y); await p.sleep(900); await p.shot(OUT + '/studio-inspector-process.png'); await p.close();
  p = await open(D + '/outer_system.rossystem', 1200, 520, 'operator'); await fit(p);
  const c = await p.where('text', '−'); console.log(JSON.stringify(c)); await p.clickAt(c.x, c.y); await p.sleep(1500); await p.shot(OUT + '/studio-subsystem-collapsed.png'); await p.close();
  console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
