const { open, fit, D, OUT } = require('./lib.js');
(async () => {
  const p = await open(D + '/ac_system.rossystem', 1200, 700, 'planner'); await fit(p);
  await p.click('button', 'Auto-connect'); await p.waitFor(`document.querySelector('.rt-ac-panel') && document.querySelector('.rt-ac-panel').innerText.length>80`, 40000); await p.sleep(1500);
  await p.shot(OUT + '/studio-autoconnect.png'); await p.close(); console.log('done');
})().catch((e) => { console.error(e); process.exit(1); });
