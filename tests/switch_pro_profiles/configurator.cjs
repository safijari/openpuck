const fs=require('fs'),path=require('path'),assert=require('assert');
const {JSDOM}=require('jsdom');
const html=fs.readFileSync(path.join(__dirname,'../../docs/index.html'),'utf8');
let cleanup=()=>{};
(async()=>{
const dom=new JSDOM(html,{url:'http://localhost/',runScripts:'outside-only'}),w=dom.window;cleanup=()=>w.close();w.TextEncoder=TextEncoder;w.TextDecoder=TextDecoder;w.fetch=async()=>({ok:false,json:async()=>[]});
w.eval(html.split('<script>')[1].split('</script>')[0]+"\nwindow.testConnect=()=>{dev={serialNumber:'test'};};");
w.eval("loadLizard=async()=>{};lizardCapable=()=>false;window.writes=[];setField=async(f,v)=>window.writes.push([f,v]);send=async bytes=>window.writes.push(Array.from(bytes));");
const p=Array(251).fill(0);p[0]=36;p[60]=1;p[193]=8;p[51]=100;p[233]=150;p[194]=1;p[195]=0;p[231]=1;p[232]=1;p[236]=8;p[237]=5;p[238]=0;p[239]=8;p[240]=2;p[241]=18;p[242]=63;p[243]=100;p[244]=150;p[245]=250;p[246]=100;p[247]=150;p[248]=250;p[249]=1;p[250]=0;
for(let i=0;i<7;i++)p[212+i]=i+1;
const apply=()=>w.eval('applyBlob(new Uint8Array('+JSON.stringify(p)+'))');apply();
assert(!w.document.querySelector('#calibrationCard'));assert(!w.document.querySelector('#hdWaveform'));
const styles=w.document.querySelector('#rumbleStyle');assert.deepEqual(Array.from(styles.options,o=>+o.value),[0,1,2,3,4,5,6,8]);assert.equal(styles.options[7].textContent,'HD Emulation');assert.equal(styles.value,'8');
assert.equal(w.document.querySelector('#rumbleState3').value,'8');assert.equal(w.document.querySelector('#strength01').value,'300');assert.equal(w.document.querySelector('#qamSelect').value,'18');
assert.equal(w.document.querySelector('#scQam').textContent,'Quick Access');assert.equal(w.document.querySelector('#scProfiles').textContent,'Switch profiles');
assert(w.document.querySelector('.chord').parentElement.classList.contains('hide'));
const profileChords=w.document.querySelectorAll('#swProfileChords select');assert(profileChords[3].parentElement.classList.contains('hide'));assert(!profileChords[0].parentElement.classList.contains('hide'));
w.document.querySelector('#scQam').click();assert.deepEqual(w.writes.pop(),[150,62]);p[242]=62;apply();assert.equal(w.document.querySelector('#scQam').textContent,'All four back buttons');
w.document.querySelector('#scProfiles').click();assert.deepEqual(w.writes.pop(),[150,60]);p[242]=60;apply();assert(!w.document.querySelector('.chord').parentElement.classList.contains('hide'));
w.document.querySelector('#scHaptics').click();assert.deepEqual(w.writes.pop(),[150,56]);p[242]=56;apply();for(const el of w.document.querySelectorAll('.chordD'))assert(!el.parentElement.classList.contains('hide'));
for(const [id,bit] of [['scFeedback',8],['scCapture',16],['scEnabled',32]]){w.document.querySelector('#'+id).click();assert.deepEqual(w.writes.pop(),[150,p[242]^bit]);}
const preset=w.document.querySelector('#rumbleState2');preset.value='6';preset.dispatchEvent(new w.Event('change'));assert.deepEqual(w.writes.pop(),[152,6]);
const step=w.document.querySelector('#strength12');step.value='420';step.dispatchEvent(new w.Event('change'));assert.deepEqual(w.writes.pop(),[159,210]);let before=w.writes.length;step.value='421';step.dispatchEvent(new w.Event('change'));assert.equal(w.writes.length,before);
const capture=w.document.querySelector('#qamSelect');capture.value='0';capture.dispatchEvent(new w.Event('change'));assert.deepEqual(w.writes.pop(),[149,0]);
// Original per-type mapping, mode and profile controls remain present.
assert.equal(w.document.querySelectorAll('.modebtn').length,11);assert.equal(w.document.querySelectorAll('#swProfileBack select').length,4);assert.equal(profileChords.length,7);
assert(w.document.querySelector('#lizardList'));assert(w.document.querySelector('#swGyroMap'));assert(w.document.querySelector('#swClickFeedback'));
const edit=w.document.querySelector('#swProfileEdit');edit.value='6';edit.dispatchEvent(new w.Event('change'));const backs=w.document.querySelectorAll('#swProfileBack select');backs[3].value='20';backs[3].dispatchEvent(new w.Event('change'));assert.deepEqual(w.writes.pop(),[139,20]);
p[242]=63;apply();w.testConnect();w.confirm=()=>true;
const bp=Array(98).fill(0);bp[0]=1;w.eval("readBlob=async()=>new Uint8Array("+JSON.stringify(p)+");readFrame=async()=>new Uint8Array("+JSON.stringify(bp)+");waitIdle=async()=>{};modalOpen=()=>{};modalStage=()=>{};modalDone=()=>{};");
const backup=w.eval('buildBackup('+JSON.stringify(p)+','+JSON.stringify(bp)+')');assert.deepEqual(Array.from(backup.config.rumblePresets),[5,0,8]);assert.deepEqual(Array.from(backup.config.strengthSteps),[200,300,500,200,300,500]);assert.equal(backup.config.shortcutFlags,63);assert(!('hdWaveform' in backup.config));assert(!('waveToggle' in backup.config));
await w.importBackup({text:async()=>JSON.stringify(backup)});for(const [f,v] of [[151,5],[153,8],[154,100],[159,250],[150,63],[149,18],[161,1],[162,0]])assert(w.writes.some(x=>x[0]===2&&x[1]===f&&x[2]===v),JSON.stringify({f,v,log:w.document.querySelector("#log").textContent,backup:backup.config}));
const bad=JSON.parse(JSON.stringify(backup));bad.config.strengthSteps[0]=301;before=w.writes.length;await w.importBackup({text:async()=>JSON.stringify(bad)});assert.equal(w.writes.length,before);
p[0]=35;apply();before=w.writes.length;w.confirm=()=>{throw Error('must reject incompatible restore');};await w.importBackup({text:async()=>JSON.stringify(backup)});assert.equal(w.writes.length,before);
console.log('Final configurator controls, legacy features, toggles and backup tests passed');w.close();
})().catch(e=>{cleanup();console.error(e);process.exitCode=1;});
