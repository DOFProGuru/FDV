import { defineConfig } from 'vite';

/**
 * The base is relative on purpose, and it is the whole reason there is a config file at all.
 *
 * Vite's default is `/`, which bakes root-absolute URLs into the bundle (`/assets/index-…js`,
 * `/favicon.svg`). That serves fine from `npm run preview` and from a user page, and 404s on a
 * project page, which lives under `/<repo>/`. Pinning an absolute base instead (`/FDV/`) fixes the
 * project page and breaks the moment the site moves somewhere else.
 *
 * With `./` the same bytes serve from anywhere over http(s) — a project page, a domain root, a
 * subdirectory of an existing site — so the deploy is portable and the build is not
 * environment-dependent. It holds because nothing else in the app reaches for an absolute path: the
 * bundled flights are fetched as `data/index.json` relative to the document (src/ui/load.ts), and
 * there is no router, so no deep link can resolve against the wrong directory. A router with real
 * routes would need an absolute base, or a `<base href>`, because relative URLs would then be
 * measured from the route rather than from the site root.
 *
 * One thing this does not buy: `file://` still fails, because the module script is refused from a
 * `null` origin before the base has any say in it. `npm run preview` remains how to look at a build
 * locally. Both paths are verified with `node tools/smoke.mjs --url …` against one `dist/` served at
 * `/` and at `/FDV/` — the two cases the bundle now has to survive.
 */
export default defineConfig({
  base: './',
});
