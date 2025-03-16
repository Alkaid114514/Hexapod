let ws = null;

document.getElementById('connectBtn').addEventListener('click', function() {
    const addressInput = document.getElementById('serverAddress');
    const wsUrl = addressInput.value.trim();

    // 输入验证
    if (!wsUrl) {
        appendMessage('错误：请输入服务器地址', 'Sensor1');
        addressInput.focus();
        return;
    }
    // i表示不区分大小写
    if (!/^wss?:\/\//i.test(wsUrl)) {
        appendMessage('错误：地址必须以 ws:// 或 wss:// 开头', 'Sensor1');
        addressInput.focus();
        return;
    }

    connectWebSocket(wsUrl);
});

function connectWebSocket(wsUrl) {
    if (ws) {
        ws.close(); // 关闭已有连接
    }

    ws = new WebSocket(wsUrl);

    // 连接状态处理
    ws.onopen = () => {
        appendMessage(`成功连接到：${wsUrl}`, 'Sensor1');
        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
    };

    // 消息处理（增强版）
    ws.onmessage = (event) => {
        handleMessage(event.data);
    };

    // 错误处理（增强版）
    ws.onerror = (error) => {
        appendMessage(`连接错误：${error.message || '未知错误'}`, 'Sensor1');
        resetConnection();
    };

    // 关闭处理（增强版）
    ws.onclose = (event) => {
        if (event.wasClean) {
            appendMessage(`连接正常关闭，代码：${event.code}`, 'Sensor1');
        } else {
            appendMessage(`连接意外中断，代码：${event.code}`, 'Sensor1');
        }
        resetConnection();
    };
}

// 断开连接处理
document.getElementById('disconnectBtn').addEventListener('click', function() {
    if (ws) {
        ws.close(1000, '用户主动断开');
    }
});

// 消息处理函数
function handleMessage(data) {
    try {
      const jsonData = JSON.parse(data);
      switch (jsonData.type) {
        case 'Sensor1':
          appendMessage(jsonData.data, 'Sensor1'); // 直接显示文本
          break;
        case 'Sensor2':
          // 格式化 GPS 数据
          const gpsInfo = [];
          if (jsonData.data.latitude) {
            gpsInfo.push(`纬度: ${jsonData.data.latitude.toFixed(6)}`);
            gpsInfo.push(`经度: ${jsonData.data.longitude.toFixed(6)}`);
            gpsInfo.push(`海拔: ${jsonData.data.altitude} 米`);
            gpsInfo.push(`速度: ${jsonData.data.speed} km/h`);
          } else {
            gpsInfo.push(jsonData.data.status);
          }
          appendMessage(gpsInfo.join('\n'), 'Sensor2');
          break;
        case 'Sensor3':
          // 显示姿态角度
          const attitudeInfo = `俯仰角: ${jsonData.data.pitch.toFixed(2)}°\n横滚角: ${jsonData.data.roll.toFixed(2)}°`;
          appendMessage(attitudeInfo, 'Sensor3');
          break;
      }
    } catch (e) {
      appendMessage(`收到非结构化消息: ${data}`);
    }
  }

// 重置连接状态
function resetConnection() {
    document.getElementById('connectBtn').disabled = false;
    document.getElementById('disconnectBtn').disabled = true;
    ws = null;
}

// 消息显示函数（增强版）
function appendMessage(message, type) {
    const messagesDiv = document.getElementById(`messages${type.charAt(0).toUpperCase() + type.slice(1)}`);
    const messageElement = document.createElement('div');
    messageElement.style.padding = '5px';
    messageElement.style.borderBottom = '1px solid #eee';
    
    // 添加时间戳
    const timestamp = new Date().toLocaleTimeString();
    messageElement.innerHTML = `
        <span style="color:#666; font-size:0.8em">[${timestamp}]</span>
        <span style="margin-left:10px">${message}</span>
    `;
    
    messagesDiv.appendChild(messageElement);
    messagesDiv.scrollTop = messagesDiv.scrollHeight;
}
