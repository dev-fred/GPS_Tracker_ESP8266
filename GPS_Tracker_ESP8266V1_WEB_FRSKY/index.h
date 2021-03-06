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
<script type="text/javascript">
setInterval(function() {
// Call a function repetatively with 1 Second interval
getData(0);
getData(1);
getData(2);
getData(3);
getData(4);
getData(5);
getData(6);
getData(7);
getData(8);
getData(9);
getData(10);
getData(11);
getData(12);
getData(13);
}, 3000); //3000mSeconds update rate
function getData(i) {
var xhttp = new XMLHttpRequest();
xhttp.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("Value"+i).innerHTML =
this.responseText;
}
};
xhttp.open("GET", "read"+i, true);
xhttp.send();
};
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
