import { defineConfig } from 'vite';

/**
 * The base is relative on purpose, and it is the whole reason there is a config file at all.
 *
 * Vite's default is `/`, which bakes root-absolute URLs into the bundle (`/assets/index-…js`,
 * `/favicon.svg`). That serves fine from `npm run preview` and from a user page, and 404s on a
 * project page, which lives under `/<repo>/`. Pinning an absolute base instead (`/FDV/`) fixes the
 * project page and breaks the moment the site moves — to a custom domain, to `/FDV` without the
 * trailing slash, or to a clone opened from the filesystem.
 *
 * With `./` the same bytes serve from any of those over HTTP, so the deploy is portable and the build
 * is not environment-dependent. Anything you can point a browser at over http(s) works: `npx serve
 * dist`, a subdirectory of an existing site, the project page, a domain later. What does not work is
 * `file://` — the module script is refused from a `null` origin before the base has any say in it, so
 * `npm run preview` remains the way to look at a build locally. Verified both ways with
 * `node tools/smoke.mjs --url …` against the same `dist/` served at `/` and at `/FDV/`. It holds because nothing else in the app reaches for an absolute path:
 * the bundled flights are fetched as `data/index.json` relative to the document (src/ui/load.ts),
 * and there is no router, so no deep link can resolve against the wrong directory. A router with
 * real routes would need an absolute base, or a `<base href>`, because relative URLs would then be
 * measured from the route rather than from the site root.
 */
export default defineConfig({
  base: './',
});
