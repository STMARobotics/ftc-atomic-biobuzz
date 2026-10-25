// Builds tools/field_map_65_80.html: minimal dark map for 65-80 deg hood, +/-500 RPM, 90% of directions.
const fs = require("fs");
const j = JSON.parse(fs.readFileSync(process.argv[2] + "/z6580.json", "utf8"));
let cls = ""; for (let i = 0; i < j.nx * j.nz; i++) { const c = j.grid.cls[i]; cls += c < 0 ? "x" : c; }
const d = { nx: j.nx, nz: j.nz, x0: j.x0, z0: j.z0, step: j.step, cls };
fs.writeFileSync(__dirname + "/field_map_65_80.html", `<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SOTM field map, 65-80 deg hood</title>
<style>body{margin:0;padding:16px;background:#1c1c1a;color:#f1efe8;font:15px system-ui,sans-serif;max-width:640px;margin:auto}
canvas{width:100%;display:block;border-radius:8px}.lg{display:flex;flex-wrap:wrap;gap:6px 18px;margin-top:12px}.lg span{display:inline-flex;align-items:center;gap:6px}.lg i{width:14px;height:14px;border-radius:3px;display:inline-block}</style></head><body>
<canvas id="c" width="640" height="640"></canvas>
<div class="lg"><span><i style="background:#1D9E75"></i>SOTM works</span><span><i style="background:#378ADD"></i>Stationary only</span><span><i style="background:#5a5a56"></i>No shot</span></div>
<script>
const d=${JSON.stringify(d)},SZ=640,SC=SZ/144,ctx=document.getElementById("c").getContext("2d");
const COL=["#1D9E75","#1D9E75","#378ADD","#5a5a56"];
for(let i=0;i<d.nx*d.nz;i++){const x=d.x0+(i%d.nx)*d.step,z=d.z0+Math.floor(i/d.nx)*d.step,ch=d.cls[i];
ctx.fillStyle=ch==="x"?"#2a2a28":COL[+ch];ctx.fillRect((x-d.step/2)*SC,(z-d.step/2)*SC,d.step*SC+0.5,d.step*SC+0.5);}
ctx.fillStyle="#8a8a85";ctx.fillRect(47.3*SC,52.5*SC,49.4*SC,39*SC);
ctx.fillStyle="#EF9F27";ctx.beginPath();ctx.arc(59.25*SC,88*SC,6,0,7);ctx.fill();
</script></body></html>`);
console.log("ok");
