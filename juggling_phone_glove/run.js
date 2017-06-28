#!/usr/bin/env node
'use strict';

const express = require('express');
const argv = require('minimist')(process.argv)
const app = express();
const server = require('http').createServer(app);
const micros = require('microseconds');
const colorize = require('ansi-color-stream')

const io = require('socket.io')(server);

const path = require('path');
const publicFolder = path.join(__dirname, 'public');

const spawn = require('child_process').spawn

// set the 'public' folder as the default static folder for requests
app.use(express.static(publicFolder));

/*
const device = require('express-device');
app.use(device.capture());
*/

// start the ROS bridge then pipe its outputs/errors to the main process
const rosPy = spawn('python', ['-u', path.join(__dirname, 'python', 'ros_bridge.py')])

rosPy.stdout
.pipe(colorize('yellow'))
.pipe(process.stdout)

rosPy.stderr
.pipe(colorize('red'))
.pipe(process.stderr)


app.get('/', (req, res) => {
	res.redirect('phone.html')

	// check the device type
/*	if (req.device.type === 'tablet' || req.device.type === 'phone') {
		res.redirect('fun.html');
	}

	else {
		res.redirect('web.html');
	}*/
});

io.on('connection', (socket) => {
	socket.on('glove_input', (glove_input) => {
		glove_input.micros_tms = micros.now()
/*		console.log(glove_input)*/
		rosPy.stdin.write(JSON.stringify(glove_input) + '\n')
	});
})

io.on('debug', (data) => {
	console.log('[DEBUG]', data)
})
io.on('error', (data) => {
	console.log('[ERROR]', data)
})

const port = argv.p || 3000

server.listen(port, (err) => {
	if (err)
		console.error(err)
	else
		console.log('Listening on port', port)
});
