// Runs the real RosTooling extension code (RosCustomEditorProvider, GlspClientBridge and the
// built webview) outside VS Code, so the Studio can be opened in a browser. The `vscode` module is
// a minimal stand-in; the model server is the real Java server.
const http = require('http'), fs = require('fs'), path = require('path'), os = require('os'), Module = require('module');
const EXT = process.env.EXT_ROOT, WS = process.env.WS_ROOT, PORT = Number(process.env.PORT || 8790);
const JAR = process.env.ROSMODEL_JAR, GLSP_PORT = Number(process.env.GLSP_PORT || 5019);
const log = [];

const mkUri = (p) => ({ fsPath: p, path: p, scheme: 'file', authority: '', toString: () => 'file://' + p, with() { return this; } });
class Position { constructor(l, c) { this.line = l; this.character = c; } }
class Range { constructor(s, e) { this.start = s; this.end = e; } }
class WorkspaceEdit { constructor() { this.edits = []; } replace(uri, range, text) { this.edits.push({ uri, range, text }); } }
class Disposable { constructor(f) { this.f = f; } dispose() { if (this.f) this.f(); } }
const changeListeners = [], saveListeners = [], docs = new Map();

function makeDoc(file) {
  if (docs.has(file)) return docs.get(file);
  let text = fs.readFileSync(file, 'utf8');
  const lineStarts = () => { const s = [0]; for (let i = 0; i < text.length; i++) if (text[i] === '\n') s.push(i + 1); return s; };
  const doc = {
    uri: mkUri(file), fileName: file, languageId: path.extname(file).slice(1), isDirty: false, version: 1,
    getText() { return text; },
    positionAt(o) { const s = lineStarts(); let l = 0; while (l + 1 < s.length && s[l + 1] <= o) l++; return new Position(l, o - s[l]); },
    offsetAt(p) { return lineStarts()[p.line] + p.character; },
    save() { return Promise.resolve(true); },
    _set(t) { text = t; fs.writeFileSync(file, t); doc.version++; },
  };
  docs.set(file, doc);
  return doc;
}

const vscode = {
  Uri: { file: mkUri, joinPath: (u, ...s) => mkUri(path.join(u.fsPath, ...s)), parse: (s) => mkUri(String(s).replace('file://', '')) },
  Position, Range, WorkspaceEdit, Disposable,
  EventEmitter: class { constructor() { this.l = []; this.event = (f) => { this.l.push(f); return new Disposable(); }; } fire(e) { this.l.forEach((f) => f(e)); } dispose() {} },
  ProgressLocation: { Notification: 15 }, ExtensionMode: { Production: 1, Development: 2, Test: 3 }, FileType: { File: 1, Directory: 2 },
  workspace: {
    workspaceFolders: [{ uri: mkUri(WS), name: 'ws', index: 0 }],
    get textDocuments() { return [...docs.values()]; },
    getWorkspaceFolder: () => ({ uri: mkUri(WS), name: 'ws', index: 0 }),
    onDidChangeTextDocument: (f) => { changeListeners.push(f); return new Disposable(); },
    onDidSaveTextDocument: (f) => { saveListeners.push(f); return new Disposable(); },
    onDidChangeConfiguration: () => new Disposable(),
    getConfiguration: () => ({ get: (k, d) => d, inspect: () => undefined, update: async () => {} }),
    openTextDocument: async (u) => makeDoc(u.fsPath || String(u)),
    applyEdit: async (we) => {
      const byDoc = new Map();
      for (const e of we.edits) { const d = makeDoc(e.uri.fsPath); (byDoc.get(d) || byDoc.set(d, []).get(d)).push(e); }
      for (const [d, es] of byDoc) {
        const withOff = es.map((e) => ({ s: d.offsetAt(e.range.start), e: d.offsetAt(e.range.end), t: e.text })).sort((a, b) => b.s - a.s);
        let t = d.getText();
        for (const x of withOff) t = t.slice(0, x.s) + x.t + t.slice(x.e);
        d._set(t);
        changeListeners.forEach((f) => f({ document: d, contentChanges: [] }));
      }
      return true;
    },
    fs: { writeFile: async (u, b) => fs.writeFileSync(u.fsPath, b), readDirectory: async () => [], stat: async () => ({}) },
    createFileSystemWatcher: () => ({ onDidChange() {}, onDidCreate() {}, onDidDelete() {}, dispose() {} }),
  },
  window: {
    activeTextEditor: undefined,
    showInformationMessage: async (m) => { log.push(['info', String(m)]); }, showWarningMessage: async (m) => { log.push(['warn', String(m)]); },
    showErrorMessage: async (m) => { log.push(['error', String(m)]); },
    withProgress: async (_o, t) => t({ report() {} }), showSaveDialog: async () => undefined, showOpenDialog: async () => undefined,
    showTextDocument: async () => undefined, createOutputChannel: () => ({ appendLine() {}, append() {}, show() {}, dispose() {} }),
  },
  commands: { executeCommand: async (c, ...a) => { log.push(['command', c, ...a.map(String)]); }, registerCommand: () => new Disposable() },
};
const orig = Module._load;
Module._load = function (req, ...rest) { return req === 'vscode' ? vscode : orig.call(this, req, ...rest); };

const { GlspClientBridge } = require(path.join(EXT, 'out/glsp/GlspClientBridge.js'));
const { RosCustomEditorProvider } = require(path.join(EXT, 'out/editor/RosCustomEditorProvider.js'));
GlspClientBridge.getInstance({ jarPath: JAR, javaExecutable: 'java', port: GLSP_PORT, cataloguePath: path.join(os.homedir(), '.rostooling', 'catalogue_repos'), autoSpawn: true });
const context = { extensionUri: mkUri(EXT), extensionPath: EXT, extensionMode: 3, subscriptions: [], globalStorageUri: mkUri(path.join(WS, '.global')), asAbsolutePath: (p) => path.join(EXT, p) };
const provider = new RosCustomEditorProvider(context);

const panels = new Map(); let nextId = 1;
const SHIM = `<script>(function(){var id=location.pathname.split('/')[2];var es=new EventSource('/p/'+id+'/events');var st;
window.acquireVsCodeApi=function(){return{postMessage:function(m){fetch('/p/'+id+'/post',{method:'POST',body:JSON.stringify(m)});},getState:function(){return st;},setState:function(s){st=s;}};};
es.onmessage=function(e){window.dispatchEvent(new MessageEvent('message',{data:JSON.parse(e.data)}));};})();</script>`;

async function openPanel(file) {
  const id = nextId++; const handlers = []; const clients = []; const queue = [];
  const send = (m) => { const s = 'data: ' + JSON.stringify(m) + '\n\n'; if (clients.length) clients.forEach((c) => c.write(s)); else queue.push(s); };
  const panel = {
    active: true, visible: true, onDidChangeViewState: () => new Disposable(), onDidDispose: () => new Disposable(), reveal() {}, dispose() {},
    webview: {
      options: {}, html: '', cspSource: `http://127.0.0.1:${PORT}`,
      asWebviewUri: (u) => ({ toString: () => `http://127.0.0.1:${PORT}/ext/` + path.relative(EXT, u.fsPath) }),
      postMessage: async (m) => { send(m); return true; }, onDidReceiveMessage: (f) => { handlers.push(f); return new Disposable(); },
    },
  };
  panels.set(id, { panel, handlers, clients, queue });
  docs.delete(file); await provider.resolveCustomTextEditor(makeDoc(file), panel, { isCancellationRequested: false });
  return id;
}

http.createServer(async (req, res) => {
  const u = new URL(req.url, 'http://x'); const m = u.pathname.match(/^\/p\/(\d+)\/(.*)$/);
  try {
    if (u.pathname === '/open') { const id = await openPanel(u.searchParams.get('file')); res.writeHead(302, { Location: `/p/${id}/` }); return res.end(); }
    if (u.pathname === '/log') { res.writeHead(200, { 'Content-Type': 'application/json' }); return res.end(JSON.stringify(log.splice(0))); }
    if (u.pathname.startsWith('/ext/')) {
      const f = path.join(EXT, u.pathname.slice(5)); if (!f.startsWith(EXT) || !fs.existsSync(f)) { res.writeHead(404); return res.end(); }
      res.writeHead(200, { 'Content-Type': f.endsWith('.js') ? 'text/javascript' : f.endsWith('.css') ? 'text/css' : 'application/octet-stream' }); return res.end(fs.readFileSync(f));
    }
    if (m) {
      const p = panels.get(Number(m[1])); if (!p) { res.writeHead(404); return res.end(); }
      if (m[2] === '') {
        const html = p.panel.webview.html.replace(/<meta http-equiv="Content-Security-Policy"[^>]*>/, '').replace('<head>', '<head>' + SHIM);
        res.writeHead(200, { 'Content-Type': 'text/html' }); return res.end(html);
      }
      if (m[2] === 'events') { res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache' }); p.clients.push(res); p.queue.splice(0).forEach((s) => res.write(s)); return; }
      if (m[2] === 'post') { let b = ''; req.on('data', (c) => (b += c)); req.on('end', () => { try { p.handlers.forEach((h) => h(JSON.parse(b))); } catch (e) { console.error(e); } res.writeHead(204); res.end(); }); return; }
    }
    res.writeHead(404); res.end();
  } catch (e) { console.error(e); res.writeHead(500); res.end(String(e)); }
}).listen(PORT, '127.0.0.1', () => console.log('studio harness on ' + PORT));
