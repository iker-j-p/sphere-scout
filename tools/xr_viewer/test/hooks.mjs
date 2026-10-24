// Node has no import maps: resolve the bare 'three' specifier to the vendored copy.
import { pathToFileURL } from 'node:url';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
const three = pathToFileURL(path.join(path.dirname(fileURLToPath(import.meta.url)),
  '../web/vendor/three/build/three.module.js')).href;
export async function resolve(specifier, context, next) {
  if (specifier === 'three') return { url: three, shortCircuit: true };
  return next(specifier, context);
}
