#pragma once

// Settings page served at GET /. Reads and writes /config; values are set via
// input.value or textContent, never innerHTML, so stored strings and API data
// can't inject markup.
inline constexpr char kSettingsPage[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ADS-B monitor</title>
<style>
body{font-family:system-ui,sans-serif;max-width:32rem;margin:1.5rem auto;padding:0 1rem;color:#222;background:#fafafa}
h1{font-size:1.3rem;margin-bottom:.2rem}
#info{color:#777;font-size:.85rem;margin-bottom:1.2rem}
fieldset{border:1px solid #ddd;border-radius:6px;margin:0 0 1rem;padding:.6rem 1rem 1rem;background:#fff}
legend{font-weight:600;padding:0 .3rem}
label{display:block;margin-top:.7rem;font-size:.9rem}
input,select,textarea{width:100%;box-sizing:border-box;padding:.45rem;margin-top:.2rem;font-size:1rem;border:1px solid #ccc;border-radius:4px}
small{color:#777}
button{padding:.6rem 1.4rem;font-size:1rem;border:0;border-radius:4px;background:#2563eb;color:#fff;cursor:pointer}
#status{margin-left:.8rem}
textarea{font:.8rem/1.4 ui-monospace,Menlo,monospace;min-height:12rem}
table{width:100%;border-collapse:collapse;font-size:.85rem;background:#fff}
td,th{text-align:left;padding:.25rem .4rem;border-bottom:1px solid #eee}
</style></head><body>
<h1>ADS-B monitor</h1>
<div id="info">&nbsp;</div>
<form id="f">
<fieldset><legend>Device</legend>
<label>Hostname<input name="hostname" required pattern="[A-Za-z0-9]([A-Za-z0-9-]{0,30}[A-Za-z0-9])?">
<small>Changing it restarts the monitor; it then answers as &lt;name&gt;.mianos.com</small></label>
<label>Brightness (%)<input name="brightness" type="number" min="1" max="100" required></label>
<label>Timezone (POSIX TZ)<input name="tz" required>
<small>Sydney: AEST-10AEDT,M10.1.0,M4.1.0/3</small></label>
</fieldset>
<fieldset><legend>Location (centre of the radar)</legend>
<label>Latitude<input name="latitude" inputmode="decimal" required><small>South is negative</small></label>
<label>Longitude<input name="longitude" inputmode="decimal" required><small>East is positive</small></label>
<label>Radius (nm)<input name="radius_nm" type="number" min="1" max="250" required>
<small>Aircraft within this distance are fetched; it is also the radar's outer ring. Tap empty radar to zoom in.</small></label>
<label>On the ground<select name="ground"><option value="1">Group into count badges</option><option value="2">Show each one</option><option value="0">Hide</option></select>
<small>Parked aircraft crowd airports. Grouped badges split apart as you zoom in.</small></label>
</fieldset>
<fieldset><legend>Data source</legend>
<label>Provider<select name="provider"></select></label>
<label>Providers (JSON)<textarea name="providers" spellcheck="false"></textarea>
<small>Each entry: "url" with {lat} {lon} {nm} filled in, "poll_s" seconds between requests (default 5),
optional "headers" such as {"api-auth":"key"}. For a local receiver use http://&lt;host&gt;/data/aircraft.json.
adsb.lol allows 500 requests a day without a key. Headers are visible to anyone who can open this page.</small></label>
</fieldset>
<button>Save</button><span id="status"></span>
</form>
<p><button type="button" id="rb" style="background:#6b7280">Restart</button></p>
<h2 style="font-size:1.1rem">Aircraft now</h2>
<table><thead><tr><th>Flight</th><th>Type</th><th>Alt ft</th><th>nm</th><th>Brg</th></tr></thead><tbody id="ac"></tbody></table>
<script>
const f=document.getElementById('f'),st=document.getElementById('status');
const ints=['radius_nm','brightness','ground'];
const pv=f.elements.provider,pt=f.elements.providers;
function table(){try{const t=JSON.parse(pt.value);return t&&typeof t==='object'&&!Array.isArray(t)?t:null;}catch(e){return null;}}
function options(want){const t=table();if(!t)return;const keep=want??pv.value;pv.replaceChildren();
 for(const k of Object.keys(t)){const o=document.createElement('option');o.value=o.textContent=k;pv.appendChild(o);}
 if(Object.keys(t).includes(keep))pv.value=keep;}
pt.addEventListener('input',()=>options());
function fill(c){for(const[k,v]of Object.entries(c)){if(k==='provider'||!f.elements[k])continue;
 if(k==='providers'){let t=v;try{t=JSON.stringify(JSON.parse(v),null,1);}catch(e){}pt.value=t;}else f.elements[k].value=v;}
 options(c.provider);}
fetch('/config').then(r=>r.json()).then(fill).catch(()=>st.textContent='Could not load settings');
fetch('/healthz').then(r=>r.json()).then(h=>{document.getElementById('info').textContent=
 'Firmware '+h.version+' on '+h.partition+', up '+Math.round(h.uptime_s/60)+' min, data from '+h.provider+(h.feed_error?' ('+h.feed_error+')':'');}).catch(()=>{});
function loadAircraft(){fetch('/aircraft').then(r=>r.json()).then(d=>{
 const tb=document.getElementById('ac');tb.replaceChildren();
 for(const a of d.aircraft){const tr=document.createElement('tr');
  for(const v of[a.flight||a.hex,a.type,a.alt_ft??'',a.dist_nm,a.bearing]){const td=document.createElement('td');td.textContent=v;tr.appendChild(td);}
  tb.appendChild(tr);}}).catch(()=>{});}
loadAircraft();setInterval(loadAircraft,10000);
f.addEventListener('submit',async e=>{
 e.preventDefault();
 let t;try{t=JSON.parse(pt.value);}catch(err){st.textContent='Providers JSON: '+err.message;return;}
 st.textContent='Saving...';
 const body={};for(const el of f.elements){if(!el.name)continue;body[el.name]=ints.includes(el.name)?Number(el.value):el.value.trim();}
 body.providers=JSON.stringify(t);
 try{const r=await fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const j=await r.json();if(!r.ok)throw new Error(j.error||r.status);
  fill(j);st.textContent=j.rebooting?'Saved, restarting...':'Saved';
 }catch(err){st.textContent='Failed: '+err.message;}
});
document.getElementById('rb').addEventListener('click',async()=>{
 if(!confirm('Restart the monitor?'))return;
 st.textContent='Restarting...';
 try{await fetch('/reboot',{method:'POST'});}catch(e){}
 setTimeout(()=>location.reload(),8000);
});
</script></body></html>)HTML";
