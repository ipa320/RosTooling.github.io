const { open, fit, D, OUT } = require('./lib.js');
(async () => {
  let p = await open(D + '/overview_system.rossystem', 1200, 460, 'scan_det'); await fit(p);
  await p.shot(OUT + '/studio-system.png'); await p.close();
  p = await open(D + '/robot_system.rossystem', 1200, 700, 'scan_in'); await fit(p);
  await p.click('#btnOpenCatalogue'); await p.sleep(1500); await p.shot(OUT + '/studio-catalogue.png');
  await p.click('.rt-drawer-tabs button', 'Sources'); await p.sleep(800); await p.shot(OUT + '/studio-catalogue-sources.png'); await p.close();
  p = await open(D + '/ac_system.rossystem', 1200, 640, 'planner'); await fit(p);
  await p.shot(OUT + '/studio-autoconnect-before.png');
  await p.click('button', 'Auto-connect'); await p.waitFor(`document.querySelector('.rt-ac-panel')`); await p.sleep(1500); await p.shot(OUT + '/studio-autoconnect.png'); await p.close();
  console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
