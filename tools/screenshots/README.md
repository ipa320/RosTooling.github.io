# Screenshots and GIFs of RosTooling Studio

The images of the VS Code Extension pages are taken from the real Studio: `harness.js` runs the extension's own code (`RosCustomEditorProvider`, `GlspClientBridge` and the built webview) outside VS Code, with a minimal stand-in for the `vscode` module and the real Java model server. The Studio is opened in headless Chrome, driven by `cdp.js` (no npm packages needed), and `lib.js` has the helpers. `s*.js` take the screenshots, `g*.js` record the GIFs (with ImageMagick `convert`).

Needs: Node 20, Java 21, Google Chrome, ImageMagick, a built extension (`npm run compile` in `rostooling-languages`) and a server JAR.

```bash
export EXT_ROOT=<RosTooling_Extension>/rostooling-languages
# 1. a working copy of the sample models (the Studio writes into them)
cp -r tools/screenshots/workspace tools/screenshots/work
# 2. the harness
WS_ROOT=$PWD/tools/screenshots/work ROSMODEL_JAR=$EXT_ROOT/server/rostooling_extension-2.5.0.jar PORT=8790 \
  node tools/screenshots/harness.js &
# 3. the images (D is the folder with the models to open)
cd tools/screenshots && D=$PWD/work/src/robot_demo OUT=../../vscode/images node s1.js
```

The pages open at `http://127.0.0.1:8790/open?file=<absolute path of a model>`. The dark theme is the default of the Studio outside VS Code. Take new images when the Studio changes, and check the text of the pages against them.
