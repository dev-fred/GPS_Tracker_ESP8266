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
</style>
<title>Balise</title>
</head>
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
