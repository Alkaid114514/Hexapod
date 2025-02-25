const socket = new WebSocket('ws://你的服务器地址:端口'); // 例如 ws://192.168.1.100:8080

// 连接状态反馈
socket.onopen = () => {
    document.getElementById('status').textContent = '状态: 已连接';
};
socket.onerror = (error) => {
    document.getElementById('status').textContent = '状态: 连接失败';
};

// 按钮控制
document.getElementById('forward').addEventListener('mousedown', () => sendCommand('FORWARD'));
document.getElementById('forward').addEventListener('mouseup', () => sendCommand('STOP'));

document.getElementById('backward').addEventListener('mousedown', () => sendCommand('BACKWARD'));
document.getElementById('backward').addEventListener('mouseup', () => sendCommand('STOP'));

document.getElementById('left').addEventListener('mousedown', () => sendCommand('LEFT'));
document.getElementById('left').addEventListener('mouseup', () => sendCommand('STOP'));

document.getElementById('right').addEventListener('mousedown', () => sendCommand('RIGHT'));
document.getElementById('right').addEventListener('mouseup', () => sendCommand('STOP'));

document.getElementById('stop').addEventListener('click', () => sendCommand('STOP'));

// 键盘控制（WASD/方向键）
document.addEventListener('keydown', (e) => {
    switch(e.key) {
        case 'ArrowUp':
        case 'w':
            sendCommand('FORWARD');
            break;
        case 'ArrowDown':
        case 's':
            sendCommand('BACKWARD');
            break;
        case 'ArrowLeft':
        case 'a':
            sendCommand('LEFT');
            break;
        case 'ArrowRight':
        case 'd':
            sendCommand('RIGHT');
            break;
    }
});

document.addEventListener('keyup', (e) => {
    if (['ArrowUp', 'w', 'ArrowDown', 's', 'ArrowLeft', 'a', 'ArrowRight', 'd'].includes(e.key)) {
        sendCommand('STOP');
    }
});

// 发送指令到服务器
function sendCommand(cmd) {
    if (socket.readyState === WebSocket.OPEN) {
        socket.send(cmd);
    }
}