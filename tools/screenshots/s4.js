const { open, fit, D, OUT } = require('./lib.js');
(async () => {
  let p = await open(D + '/scanner_pkg.ros2', 1200, 760, 'laser_driver'); await p.shot(OUT + '/studio-component.png');
  await p.click('.rt-node-card, [class*="node-card"], [class*="rt-card"]', 'scan_filter').catch(() => {}); await p.sleep(800); await p.shot(OUT + '/studio-component-selected.png'); await p.close();
  p = await open(D + '/my_msgs.ros', 1200, 640, 'Hello'); await p.shot(OUT + '/studio-comm-objects.png'); await p.close();
  p = await open(D + '/proc_system.rossystem', 1200, 520, 'scan_det'); await fit(p); await p.shot(OUT + '/studio-process.png'); await p.close();
  p = await open(D + '/outer_system.rossystem', 1200, 520, 'operator'); await fit(p); await p.shot(OUT + '/studio-subsystem.png'); await p.close();
  p = await open(D + '/ac_system.rossystem', 1200, 940, 'planner'); await fit(p);
  await p.click('button', 'Auto-connect'); await p.waitFor(`document.querySelector('.rt-ac-panel') && document.querySelector('.rt-ac-panel').innerText.length>80`, 40000); await p.sleep(1500);
  await p.shot(OUT + '/studio-autoconnect-full.png'); await p.close();
  console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
