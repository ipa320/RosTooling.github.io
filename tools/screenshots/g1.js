const { open, fit, D, OUT, Frames, portCenter } = require('./lib.js');
(async () => {
  const p = await open(D + '/ac_system.rossystem', 1200, 520, 'planner'); await fit(p);
  const f = new Frames(p, 'connect'); await f.snap(40);
  const a = await portCenter(p, 'laser', 'scan : LaserScan'), b = await portCenter(p, 'filter', 'scan_in');
  console.log(JSON.stringify([a, b]));
  await p.cmd('Input.dispatchMouseEvent', { type: 'mouseMoved', x: a.x, y: a.y }); await p.mouse('mousePressed', a.x, a.y); await f.snap(15);
  const steps = 8;
  for (let i = 1; i <= steps; i++) { await p.cmd('Input.dispatchMouseEvent', { type: 'mouseMoved', x: a.x + ((b.x - a.x) * i) / steps, y: a.y + ((b.y - a.y) * i) / steps, button: 'left', buttons: 1 }); await p.sleep(60); await f.snap(10); }
  await p.mouse('mouseReleased', b.x, b.y); await p.sleep(2500); await f.snap(40); await f.snap(120);
  f.build(OUT + '/studio-connect.gif'); await p.shot(OUT + '/studio-connected.png'); await p.close(); console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
