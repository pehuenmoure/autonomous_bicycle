var express = require('express');
var app = express();
var http = require('http');
var server = http.Server(app);
var io = require('socket.io')(server);
var SerialPort = require('serialport');

//For Writing Data
var fileWriter = require('msexcel-builder');
var testNo = 0;
var name = new String("data");
var fileName = name.concat(String(testNo)).concat('.csv');
var row = 1;
var col = 1;
console.log(fileName);

//For getting input- Will be edited out once site is nicer
var prompt = require('prompt');
prompt.start();
// setPrompt();

//======================rest of code=======================
app.set('view engine', 'ejs');

if(!process.argv[2]){
	console.error("Usage: Node " + process.argv[1] + " serialport needed");
	process.exit(1);
};

var myPort = new SerialPort(process.argv[2], {
	parser: SerialPort.parsers.readline('\r\n')
});

io.on('connect', function(socket){
	console.log('a user connected');
});

var data = createArray(1000,4);
var i = 0;
myPort.on('data', function(d){
	console.log(d)
	var reading = parseFloat(d);

	console.log('reading:',reading);
	if(reading<10000){
		data[row][Math.floor(Math.trunc(reading)/1000)-1] = reading%1000;
		console.log('row',row,data[row]);
	}
	else {
		row++;
	}
});
//=======================FOR WRITING TO FILE======================
function writeToFile(){
	row=0;
		//Set up new file
	fileName = name.concat(String(testNo)).concat('.csv');
	var workbook = fileWriter.createWorkbook('./', fileName);
	var sheet = workbook.createSheet('sheet1',6,1000);
	sheet.set(1,1,'potato');
	for(var i = 0;i<data.length;i++){
		for(var j = 0;j<data[i].length;j++){
			console.log(j+1);
			console.log(i+1);
			console.log(data[i][j]);
			sheet.set(j+1,i+1,data[i][j]);
		}
	}

	workbook.save(function(ok){
		if (!ok) workbook.cancel();
	});

}

//=========================FOR INPUTTING DATA TO ARDUINO====================
// function setPrompt(){
// 	prompt.get(['input'], function (err, result) {
// 		if (err) { return onErr(err); }
// 		console.log('Command-line input received: ' + result.input);
// 		if(result.input==='start'){
// 			myPort.write('y');
// 			setPrompt();
// 		}else if(result.input==='stop'){
// 			writeToFile();
// 			//stop loop and increment testNo
// 			myPort.write('n');
// 			testNo++;
// 			setPrompt();
// 		}
// 	});
// }

function onErr(err) {
	console.log(err);
	return 1;
}

function createArray(length) {
	var arr = new Array(length || 0),
	i = length;

	if (arguments.length > 1) {
		var args = Array.prototype.slice.call(arguments, 1);
		while(i--) arr[length-1 - i] = createArray.apply(this, args);
	}

	return arr;
}


function writeAndDrain (data, callback) {
	sp.write(data, function () {
		sp.drain(callback);
	});
}


app.get('/', function(req, res){
	res.render('index');
});

server.listen(8000, function(){
	console.log('listening on 8000');
});


// var SerialPort = require('serialport');
// SerialPort.list(function (err, ports) {

// 	ports.forEach(function(port) {

// 		console.log(port.comName);
// 		console.log(port.pnpId);
// 		console.log(port.manufacturer);
// 		if(port.manufacturer.includes('Arduino')){
// 			var myPort = new SerialPort(port.comName, {
// 				parser: SerialPort.parsers.readline('\n')
// 			});


// 			io.on('connect', function(socket){
// 				console.log('a user connected');
// 			});
// 			setPrompt();

// 			var data = createArray(1000,4);
// 			var i = 0;
// 			myPort.on('data', function(d){
// 				var reading = parseFloat(d);

// 				console.log('reading:',reading);
// 				if(reading<10000){
// 					data[row][Math.floor(Math.trunc(reading)/1000)-1] = reading%1000;
// 					console.log('row',row,data[row]);
// 				}
// 				else {
// 					row++;
// 				}
// 			});
// 			//=======================FOR WRITING TO FILE======================
// 			function writeToFile(){
// 				row=0;
//  	 			//Set up new file
//   				fileName = name.concat(String(testNo)).concat('.csv');
//   				var workbook = fileWriter.createWorkbook('./', fileName);
//   				var sheet = workbook.createSheet('sheet1',6,1000);
//   				sheet.set(1,1,'potato');
//   				for(var i = 0;i<data.length;i++){
//   					for(var j = 0;j<data[i].length;j++){
//   						console.log(j+1);
//   						console.log(i+1);
//   						console.log(data[i][j]);
//   						sheet.set(j+1,i+1,data[i][j]);
//   					}
//   				}

//   				workbook.save(function(ok){
//   					if (!ok) 
//   						workbook.cancel();
//   					});

//   				}
// 			//=========================FOR INPUTTING DATA TO ARDUINO====================
// 			function setPrompt(){
// 				prompt.get(['input'], function (err, result) {
// 					if (err) { return onErr(err); }
// 					console.log('Command-line input received:');
// 					console.log(result.input);
// 					if(result.input==='start'){
// 						myPort.write('y');
// 						setPrompt();
// 					}else if(result.input==='stop'){
// 						writeToFile();
// 						//stop loop and increment testNo
// 						myPort.write('n');
// 						testNo++;
// 						setPrompt();
// 					}
// 				});
// 			}

// 			function onErr(err) {
// 				console.log(err);
// 				return 1;
// 			}
			

// 		}
// 	});
// });

