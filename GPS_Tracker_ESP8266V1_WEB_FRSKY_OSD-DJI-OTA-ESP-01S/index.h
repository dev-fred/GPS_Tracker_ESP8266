char webpage[] PROGMEM = R"=====(
<!doctype html>
<meta charset="utf-8">
<head>
<style type="text/css">
#map{height:1000px;}
.card{
height: 380px;
padding:5px;
width:100%;
background: #1f98bd81;
border-radius: 10px;
color: #000;
margin:0px;
font-size: 300%;
line-height: 0%;
}
.button {
  background-color: #f44336; /* Red */
  border: none;
  color: black;
  text-align: center;
  text-decoration: none;
  font-size:190%;
  border-radius: 10px;
  font-weight: bold;
  width: 200px;
  height: 50px;
}
</style>
<title>Balise</title>
</head>
<button class="button" onclick="document.location='/OTA_'">MaJ OTA</button>
<script>
setInterval(function() {
getData();
},3000);//3000mSeconds update rate
function getData(){
var xhttp=new XMLHttpRequest();
xhttp.onreadystatechange=function() {
// chaine reçue numero$valeur;numero$valeur ...
if (this.readyState==4&&this.status==200){
this.responseText.split(';').forEach((item)=>{
var i=item.split('$')[0].trim();
var val=item.split('$')[1].replace(/"/g,'').trim();
document.getElementById("Value"+i).innerHTML=val;
});
}
};
xhttp.open("GET","readValues",true);
xhttp.send();
}
getData();
</script>
<body onload="javascript:init()">
<div class="card">

<p><span id="Value0">0</span></h6><p>
<p><span id="Value7">0</span></h5><p>
<table border="1" style="line-height: 100%;">
<tr>
<td><span id="Value4">0</span></td>
<td><span id="Value5">0</span></td>
<td><span id="Value6">0</span></td>
</tr>
<tr>
<td><span id="Value8">0</span></td>
<td><span id="Value2">0</span></td>
<td><span id="Value3">0</span></td>
</tr>
<td><span id="Value1">0</span></td>
<td><span id="Value9"></span></td>
<td><span id="Value13"></span></td>
</tr>
<td><span id="Value10">ATTENTE</span></td>
<td><span id="Value11"></span></td>
<td><span id="Value12"></span></td>
</table>
</div>
</body>
</html>
)=====";

char pageOTA[] PROGMEM = R"=====(
<!doctype html>
<meta charset="utf-8">
<head>
<style type="text/css">
#map{height:1000px;}
.card{
padding:5px;
width:100%;
height:200%;
background: #1f98bd81;
border-radius: 10px;
color: #000;
margin:10px;
font-size:4vw;
/*font-size: 300%;*/
}
input,button{
  padding:20px;
    text-align: center;
//  height:200%;
  margin:10px;
font-size:inherit;
border:5px solid  black ;
border-radius:10px;
}
</style>
<title>OTA Balise</title>
</head>
<body>
<form class="card" method='POST' id='upload_form' enctype='multipart/form-data'>
<input type='file' name='update' >
<input type='button' onclick='uploadFile()' style='background-color:red;' value='Update'>
</form>
<div>
<progress id="bar" class="card" value="0" max="100"></progress>
</div>
<div id="res" class="card"></div>
<script>
function gel(id){return document.getElementById(id);}
function uploadFile() {
gel('res').innerHTML = "Attendre un message ERREUR ou OK !!";
var form=gel('upload_form');
var data=new FormData(form);
var xhr=new window.XMLHttpRequest();
xhr.upload.addEventListener('progress',h_pro,false);
xhr.upload.addEventListener('loadstart',h_loadstart,false);
xhr.upload.addEventListener('abort',h_abort,false);
xhr.upload.addEventListener('error',h_error,false);
xhr.upload.addEventListener('load',h_load,false);
xhr.upload.addEventListener('timeout',h_timeout,false);
xhr.upload.addEventListener('loadend',h_pro,false);
xhr.open('POST','/update');
xhr.send(data);
xhr.onreadystatechange=function(){
gel('bar').value=0;
if(xhr.readyState==4)gel('res').innerHTML=xhr.responseText;
};
}
function h_loadstart(evt){console.log("h_loadstart");}
function h_abort(evt){alert("abort");}
function h_error(evt){alert("h_error");gel('res').innerHTML ="ERREUR !!";}
function h_load(evt){console.log("load");}
function h_timeout(evt){alert("h_timeout");}
function h_loadend(evt){alert("h_loadend");}
function h_pro(evt){
console.log("pro");
if (evt.lengthComputable){
var per=evt.loaded/evt.total;
gel('bar').value=Math.round(per*100) ;
}
}
</script>
</body>
</html>
)=====";
