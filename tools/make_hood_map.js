// Builds tools/hood_range_map.html (standalone page to send to CAD) from sotm_zones.js JSON output.
const fs = require("fs"), dir = process.argv[2];
const sets = { strict: { "60-75": "z6075", "65-80": "z6580" }, relaxed: { "60-75": "w_60_75", "65-80": "w_65_80" } };
const D = {};
for (const m in sets) { D[m] = {}; for (const r in sets[m]) {
  const j = JSON.parse(fs.readFileSync(dir + "/" + sets[m][r] + ".json", "utf8")); let cls = "", fh = "";
  for (let i = 0; i < j.nx * j.nz; i++) { const c = j.grid.cls[i]; cls += c < 0 ? "x" : c; fh += Math.min(9, Math.round(j.grid.fHood[i] * 9)); }
  D[m][r] = { nx: j.nx, x0: j.x0, z0: j.z0, step: j.step, nz: j.nz, cls, fh, presets: j.presets };
} }
fs.writeFileSync(__dirname + "/hood_range_map.html", fs.readFileSync(__dirname + "/hood_range_map.tpl.html", "utf8").replace("/*DATA*/", JSON.stringify(D)));
console.log("ok");
