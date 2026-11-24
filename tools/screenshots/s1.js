const { open, fit, D, OUT } = require('./lib.js');
(async () => {
  let p = await open(D + '/robot_system.rossystem', 1200, 520, 'scan_in'); await fit(p);
  await p.shot(OUT + '/studio-system.png');
  await p.click('.ros-node', 'filter'); await p.sleep(800); await p.shot(OUT + '/studio-inspector-node.png');
  await p.close();
  p = await open(D + '/broken_system.rossystem', 1200, 520, 'obstacles_in'); await fit(p);
  await p.click('.rt-diag-pill'); await p.sleep(800); await p.shot(OUT + '/studio-diagnostics.png'); await p.close();
  console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
