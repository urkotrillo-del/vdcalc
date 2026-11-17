// Runs the Python core (Pyodide) off the main thread. Module worker: classic workers are not supported by Pyodide 314.
let api = null;

function clean(args) {                       // JavaScript null reaches Python as a special object: use undefined instead
  return args.map(a => (a === null ? undefined : a));
}

async function boot(indexURL, zipURL) {
  const { loadPyodide } = await import(indexURL + 'pyodide.mjs');
  postMessage({ type: 'status', text: 'Loading Python…' });
  const py = await loadPyodide({ indexURL });
  postMessage({ type: 'status', text: 'Loading NumPy and SciPy…' });
  await py.loadPackage(['numpy', 'scipy']);
  postMessage({ type: 'status', text: 'Loading the calculator…' });
  const buf = await (await fetch(zipURL, { cache: 'no-store' })).arrayBuffer();   // never reuse an old copy of the calculator
  py.unpackArchive(buf, 'zip');
  api = py.pyimport('vdcalc.webapi');
}

onmessage = async (e) => {
  const { id, fn, args, indexURL, zipURL } = e.data;
  try {
    if (fn === '__boot__') {
      await boot(indexURL, zipURL);
      postMessage({ id, ok: true });
      return;
    }
    let call = clean(args || []);
    if (fn === 'whatif') {                   // progress callback as the 5th argument
      call = [call[0], call[1], call[2], call[3], (done, total) => postMessage({ type: 'progress', id, done, total })];
    }
    const out = api[fn](...call);
    postMessage({ id, ok: true, result: out });
  } catch (err) {
    postMessage({ id, ok: false, error: String(err && err.message ? err.message : err) });
  }
};
