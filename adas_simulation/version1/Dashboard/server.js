const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const dgram = require('dgram');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// UDP Socket to receive data from QNX
const udpServer = dgram.createSocket('udp4');

const UDP_PORT = 5000;
const WEB_PORT = 3000;

// Serve static files from the 'public' directory
app.use(express.static('public'));

// Setup UDP server
udpServer.on('error', (err) => {
    console.error(`UDP Server error:\n${err.stack}`);
    udpServer.close();
});

udpServer.on('message', (msg, rinfo) => {
    const rawData = msg.toString();
    console.log(`Received from QNX [${rinfo.address}:${rinfo.port}]: ${rawData}`);
    
    try {
        // Try parsing assuming the C code will send JSON
        const parsedData = JSON.parse(rawData);
        io.emit('qnx_data', parsedData);
    } catch (e) {
        // Fallback: If not JSON, just send the raw text as a log line
        io.emit('qnx_log', { text: rawData, timestamp: new Date().toISOString() });
    }
});

udpServer.on('listening', () => {
    const address = udpServer.address();
    console.log(`📡 UDP Receiver listening on port ${address.port} for QNX C Code...`);
});

udpServer.bind(UDP_PORT);

// Setup Socket.io connections
io.on('connection', (socket) => {
    console.log('💻 New Web Client Connected');
    socket.on('disconnect', () => {
        console.log('💻 Web Client Disconnected');
    });
});

server.listen(WEB_PORT, () => {
    console.log(`🚀 Web Dashboard running at http://localhost:${WEB_PORT}`);
});
