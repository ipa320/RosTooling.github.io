const { open, fit, D, OUT } = require('./lib.js');
(async () => {
  let p = await open(D + '/proc_system.rossystem', 1200, 520, 'scan_det'); await fit(p);
  const t = await p.where('text, g', 'Process: perception'); console.log(JSON.stringify(t));
  await p.clickAt(t.x, t.y); await p.sleep(900); await p.shot(OUT + '/studio-inspector-process.png');
  const e = await p.where('.ros-edge-line'); await p.clickAt(e.x, e.y); await p.sleep(900); await p.shot(OUT + '/studio-inspector-connection.png');
  await p.close();
  p = await open(D + '/outer_system.rossystem', 1200, 520, 'operator'); await fit(p);
  console.log(await p.eval(`JSON.stringify([...document.querySelectorAll('svg text')].map(t=>t.textContent).slice(0,8))`));
  await p.close(); console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
