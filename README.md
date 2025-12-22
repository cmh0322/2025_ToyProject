<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Vision-Guided Autonomous RC System</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
    <style>
        :root {
            --primary-color: #00adb5;
            --bg-dark: #222831;
            --bg-card: #393e46;
            --text-light: #eeeeee;
            --accent-color: #ffd369;
        }

        body {
            font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            background-color: var(--bg-dark);
            color: var(--text-light);
            line-height: 1.6;
            margin: 0;
            padding: 0;
        }

        .container {
            max-width: 1000px;
            margin: 0 auto;
            padding: 40px 20px;
        }

        header {
            text-align: center;
            padding-bottom: 60px;
            border-bottom: 2px solid var(--bg-card);
        }

        h1 {
            font-size: 2.5rem;
            color: var(--primary-color);
            margin-bottom: 10px;
            text-transform: uppercase;
            letter-spacing: 2px;
        }

        .subtitle {
            font-size: 1.2rem;
            color: var(--accent-color);
            font-weight: 300;
        }

        .section {
            margin-top: 60px;
        }

        h2 {
            border-left: 5px solid var(--primary-color);
            padding-left: 15px;
            margin-bottom: 30px;
            color: var(--primary-color);
        }

        /* 아키텍처 다이어그램 스타일 */
        .architecture-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
            margin-top: 30px;
        }

        .card {
            background-color: var(--bg-card);
            padding: 25px;
            border-radius: 12px;
            transition: transform 0.3s ease;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
        }

        .card:hover {
            transform: translateY(-10px);
            border: 1px solid var(--primary-color);
        }

        .card i {
            font-size: 2rem;
            color: var(--accent-color);
            margin-bottom: 15px;
        }

        .card h3 {
            margin-top: 0;
            color: var(--primary-color);
        }

        /* CAN 프로토콜 테이블 */
        table {
            width: 100%;
            border-collapse: collapse;
            background-color: var(--bg-card);
            border-radius: 10px;
            overflow: hidden;
            margin-top: 20px;
        }

        th, td {
            padding: 15px;
            text-align: left;
            border-bottom: 1px solid var(--bg-dark);
        }

        th {
            background-color: var(--primary-color);
            color: var(--bg-dark);
        }

        tr:hover {
            background-color: rgba(0, 173, 181, 0.1);
        }

        .badge {
            display: inline-block;
            padding: 4px 10px;
            border-radius: 4px;
            font-size: 0.8rem;
            font-weight: bold;
            background-color: var(--primary-color);
            color: var(--bg-dark);
            margin-right: 5px;
        }

        .flow-container {
            background: rgba(255, 255, 255, 0.05);
            padding: 30px;
            border-radius: 15px;
            text-align: center;
        }

        footer {
            text-align: center;
            margin-top: 80px;
            padding: 20px;
            font-size: 0.9rem;
            color: #888;
        }
    </style>
</head>
<body>

<div class="container">
    <header>
        <h1>Vision-Guided RC System</h1>
        <p class="subtitle">Raspberry Pi AI Detection & Distributed CAN Control System</p>
    </header>

    <div class="section">
        <h2><i class="fas fa-sitemap"></i> System Architecture</h2>
        <div class="flow-container">
            <p><strong>[Vision AP]</strong> ➔ (UART) ➔ <strong>[Master Node]</strong> ➔ (CAN BUS) ➔ <strong>[Slave Nodes]</strong></p>
            
        </div>
        
        <div class="architecture-grid">
            <div class="card">
                <i class="fas fa-eye"></i>
                <h3>Vision AP (Pi)</h3>
                <p>MobileNet V1 기반 객체 탐지 및 중앙 오차값 계산. 실시간 프레임 분석을 통해 주행 및 추적 데이터 생성.</p>
                <span class="badge">Python</span> <span class="badge">TFLite</span>
            </div>
            <div class="card">
                <i class="fas fa-microchip"></i>
                <h3>Master (ESP8266)</h3>
                <p>시스템의 게이트웨이. UART와 CAN 통신 간의 프로토콜 변환 및 명령 배분 담당.</p>
                <span class="badge">Arduino</span> <span class="badge">CAN-Bus</span>
            </div>
            <div class="card">
                <i class="fas fa-car-side"></i>
                <h3>Move Slave (ESP32)</h3>
                <p>FreeRTOS 기반 주행 엔진. 객체 추적 인력(Attractive)과 장애물 회피 척력(Repulsive)을 결합한 알고리즘.</p>
                <span class="badge">FreeRTOS</span> <span class="badge">L298N</span>
            </div>
        </div>
    </div>

    <div class="section">
        <h2><i class="fas fa-network-wired"></i> Communication Protocol</h2>
        <table>
            <thead>
                <tr>
                    <th>CAN ID</th>
                    <th>Node</th>
                    <th>Description</th>
                    <th>Data Format</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td><code>0x123</code></td>
                    <td>Master → All</td>
                    <td>Target Error & Detection Flag</td>
                    <td>[Header][Err_X][Err_Y]</td>
                </tr>
                <tr>
                    <td><code>0x124</code></td>
                    <td>Track Node</td>
                    <td>Servo Status & Feedback</td>
                    <td>[Status][Pos_X][Pos_Y]</td>
                </tr>
                <tr>
                    <td><code>0x125</code></td>
                    <td>Detection Node</td>
                    <td>Ultrasonic Sensor Distances</td>
                    <td>[Flag][Back][Left][Right]</td>
                </tr>
            </tbody>
        </table>
    </div>

    <div class="section">
        <h2><i class="fas fa-cogs"></i> Key Modules</h2>
        <div class="architecture-grid">
            <div class="card" style="grid-column: span 1;">
                <i class="fas fa-video"></i>
                <h3>Target Tracking</h3>
                <p>2축 서보 모터(Pan/Tilt)를 이용한 타겟 추적. 객체 유실 시 스캔 모드 자동 전환.</p>
            </div>
            <div class="card" style="grid-column: span 1;">
                <i class="fas fa-shield-halved"></i>
                <h3>Avoidance System</h3>
                <p>3방향 초음파 센서 실시간 모니터링. 50cm 이내 장애물 감지 시 주행 궤적 강제 보정.</p>
            </div>
        </div>
    </div>

    <footer>
        <p>&copy; 2025 Autonomous RC Project. Created for Embedded Engineering Portfolio.</p>
    </footer>
</div>

</body>
</html>
