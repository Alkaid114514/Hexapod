let ws = null;

document.getElementById('connectBtn').addEventListener('click', function() {
    const addressInput = document.getElementById('serverAddress');
    const wsUrl = addressInput.value.trim();

    // 输入验证
    if (!wsUrl) {
        appendMessage('错误：请输入服务器地址');
        addressInput.focus();
        return;
    }
    // i表示不区分大小写
    if (!/^wss?:\/\//i.test(wsUrl)) {
        appendMessage('错误：地址必须以 ws:// 或 wss:// 开头');
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
        appendMessage(`成功连接到：${wsUrl}`);
        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
    };

    // 消息处理（增强版）
    ws.onmessage = (event) => {
        handleMessage(event.data);
    };

    // 错误处理（增强版）
    ws.onerror = (error) => {
        appendMessage(`连接错误：${error.message || '未知错误'}`);
        resetConnection();
    };

    // 关闭处理（增强版）
    ws.onclose = (event) => {
        if (event.wasClean) {
            appendMessage(`连接正常关闭，代码：${event.code}`);
        } else {
            appendMessage(`连接意外中断，代码：${event.code}`);
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
        // 尝试解析JSON数据
        const jsonData = JSON.parse(data);
        appendMessage(`收到结构化数据：${JSON.stringify(jsonData, null, 2)}`);
    } catch {
        // 普通文本处理
        appendMessage(`收到消息：${data}`);
    }
}

// 重置连接状态
function resetConnection() {
    document.getElementById('connectBtn').disabled = false;
    document.getElementById('disconnectBtn').disabled = true;
    ws = null;
}

// 消息显示函数（增强版）
function appendMessage(message) {
    const messagesDiv = document.getElementById('messages');
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