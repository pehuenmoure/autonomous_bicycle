var express = require('express');
var app = express();
var http = require('http');
var server = http.Server(app);
var io = require('socket.io')(server);
var SerialPort = require('serialport');
var fs = require('fs');
//For getting input- Will be edited out once site is nicer
var prompt = require('prompt');
prompt.start();

app.set('view engine', 'ejs');

setPrompt();

if(!process.argv[2]){
	console.error("Usage: Node " + process.argv[1] + " serialport needed");
	process.exit(1);
};

var port = new SerialPort(process.argv[2], {
	parser: SerialPort.parsers.readline('\n')
});

io.on('connect', function(socket){
	console.log('a user connected');
});

var data = new Array(1);
var row = new Array(4);

port.on('data', function(d){

	console.log(d);
	var col = Math.floor(Math.trunc(d)/1000);
	row[col-1] = d-(col*1000);
	if(col == 4)
		{
			data.push(row);
			console.log(row);
		}
});

app.get('/', function(req, res){
	res.render('index');
});

server.listen(8000, function(){
	console.log('listening on 8000');
});

//=======================FOR WRITING TO FILE======================
function dataToString(rawdata){
	var str = "";
	//Converting data to string
	for(var i = 1;i<rawdata.length;i++){
		for(var j = 0; j<rawdata[i].length;j++){
			str+=rawdata[i][j];
			if(j<rawdata[i].length-1)
				str+=',';
		}
		str+='\n';
	}
	return str;
}

function createCSVFile(data, name){
	fs.writeFile(name+'.csv', data,  function(err) {
		if (err) {
			return console.error(err);
		}
	});
}


function downloadCSV(args, csvdata) {  
	var data, filename, link;
	var csv = dataToString(csvdata);
	if (csv == null) return;

	filename = args.filename || 'export.csv';

	if (!csv.match(/^data:text\/csv/i)) {
		csv = 'data:text/csv;charset=utf-8,' + csv;
	}
	data = encodeURI(csv);

	link = document.createElement('a');
	link.setAttribute('href', data);
	link.setAttribute('download', filename);
	link.click();
}

function test(){
	var outerarray = new Array(1);
	var innerarray = new Array(4);
	for(var i = 0;i<100;i++){
		innerarray = new Array(4);
		for(var j = 0;j<4;j++){
			innerarray[j]=i;
		}
		outerarray.push(innerarray);
	}
	var string = dataToString(outerarray);
	console.log(string);
	createCSVFile(string,'potato');
}

//FOR CONVERTING BYTES TO FLOAT--------------UNUSED
function Bytes2Float32(bytes) {
	var sign = (bytes & 0x80000000) ? -1 : 1;
	var exponent = ((bytes >> 23) & 0xFF) - 127;
	var significand = (bytes & ~(-1 << 23));

	if (exponent == 128) 
		return sign * ((significand) ? Number.NaN : Number.POSITIVE_INFINITY);

	if (exponent == -127) {
		if (significand == 0) return sign * 0.0;
		exponent = -126;
		significand /= (1 << 22);
	} else significand = (significand | (1 << 23)) / (1 << 23);

	return sign * significand * Math.pow(2, exponent);
}

//=========================FOR INPUTTING DATA TO ARDUINO====================
function setPrompt(){
	prompt.get(['input'], function (err, result) {
		if (err) { return onErr(err); }
		console.log('Command-line input received:');
		console.log(result.input);
		if(result.input==='start'){
			myPort.write('y');
			setPrompt(myPort);
		}else if(result.input==='stop'){
 			//stop loop and increment testNo
 			myPort.write('n');
 			testNo++;
			setPrompt(myPort);
		}else if(result.input === 'save'){
			createCSVFile(dataToString(data),'potato');
		}
 		});
}

function onErr(err) {
	console.log(err);
	return 1;
}



