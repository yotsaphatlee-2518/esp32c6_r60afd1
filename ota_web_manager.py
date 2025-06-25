from flask import Flask, request, render_template_string, send_from_directory
import os
from datetime import datetime

app = Flask(__name__)
# ตั้งค่าให้โฟลเดอร์ build เป็นที่สำหรับเก็บและดาวน์โหลดไฟล์ .bin
UPLOAD_FOLDER = './build'

# สร้างโฟลเดอร์ build ถ้ายังไม่มี
if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

# โค้ด HTML และ CSS สำหรับสร้างหน้าเว็บ
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="th">
<head>
    <meta charset="UTF-8">
    <title>ESP32 Firmware Manager</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 2em; background-color: #f4f4f9; color: #333; }
        .container { max-width: 800px; margin: auto; padding: 2em; background-color: #fff; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #0056b3; }
        h2 { border-bottom: 2px solid #dee2e6; padding-bottom: 0.5em; }
        .section { margin-bottom: 2em; }
        a { text-decoration: none; color: #007bff; font-weight: bold; }
        a:hover { text-decoration: underline; }
        ul { list-style-type: none; padding: 0; }
        li { background-color: #e9ecef; border: 1px solid #ced4da; padding: 0.8em; margin-bottom: 0.5em; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; }
        .file-info { display: flex; flex-direction: column; }
        .file-name { font-weight: bold; }
        .file-time { font-size: 0.8em; color: #6c757d; }
        .message { padding: 1em; margin-bottom: 1em; border-radius: 5px; font-weight: bold; }
        .success { background-color: #d4edda; color: #155724; }
        .error { background-color: #f8d7da; color: #721c24; }
        input[type="file"] { border: 1px solid #ccc; padding: 5px; border-radius: 4px; }
        input[type="submit"] { background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; }
        input[type="submit"]:hover { background-color: #0056b3; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Firmware Manager</h1>
        
        {% if message %}
            <div class="message {{ 'success' if 'สำเร็จ' in message else 'error' }}">{{ message }}</div>
        {% endif %}

        <div class="section">
            <h2>อัปโหลดเฟิร์มแวร์ใหม่ (.bin)</h2>
            <form method=post enctype=multipart/form-data>
              <input type=file name=file required>
              <input type=submit value="อัปโหลดไฟล์">
            </form>
        </div>

        <div class="section">
            <h2>ดาวน์โหลดเฟิร์มแวร์ที่มีอยู่</h2>
            <p>URL สำหรับ OTA: <strong>http://192.168.1.69:8000/download/&lt;ชื่อไฟล์&gt;</strong></p>
            {% if files %}
                <ul>
                {% for file in files %}
                    <li>
                        <div class="file-info">
                            <span class="file-name">{{ file.name }}</span>
                            <span class="file-time">เวลาอัปเดต: {{ file.time }}</span>
                        </div>
                        <a href="/download/{{ file.name }}">ดาวน์โหลด</a>
                    </li>
                {% endfor %}
                </ul>
            {% else %}
                <p>ยังไม่มีไฟล์เฟิร์มแวร์ในระบบ</p>
            {% endif %}
        </div>
    </div>
</body>
</html>
"""

@app.route('/', methods=['GET', 'POST'])
def firmware_manager():
    message = None
    if request.method == 'POST':
        if 'file' not in request.files or request.files['file'].filename == '':
            message = 'ผิดพลาด: ไม่ได้เลือกไฟล์ใดๆ'
        else:
            file = request.files['file']
            if file and file.filename.endswith('.bin'):
                filename = file.filename
                # บันทึกไฟล์ไปยังโฟลเดอร์ build
                filepath = os.path.join(UPLOAD_FOLDER, filename)
                file.save(filepath)
                message = f'อัปโหลดไฟล์ "{filename}" สำเร็จ!'
            else:
                message = 'ผิดพลาด: กรุณาเลือกไฟล์ .bin เท่านั้น'

    # อ่านรายชื่อไฟล์และเวลาที่แก้ไข
    files_with_time = []
    if os.path.exists(UPLOAD_FOLDER):
        for filename in os.listdir(UPLOAD_FOLDER):
            if filename.endswith('.bin'):
                filepath = os.path.join(UPLOAD_FOLDER, filename)
                mtime_epoch = os.path.getmtime(filepath)
                mtime_readable = datetime.fromtimestamp(mtime_epoch).strftime('%Y-%m-%d %H:%M:%S')
                files_with_time.append({'name': filename, 'time': mtime_readable})
    
    # เรียงลำดับไฟล์ตามเวลาล่าสุดขึ้นก่อน
    files_with_time.sort(key=lambda x: x['time'], reverse=True)
    
    return render_template_string(HTML_TEMPLATE, files=files_with_time, message=message)

@app.route('/download/<path:filename>')
def download_file(filename):
    # ฟังก์ชันสำหรับส่งไฟล์ให้ ESP32 หรือ browser ดาวน์โหลดจากโฟลเดอร์ build
    # แก้ไข: เปลี่ยน as_attachment=True เป็น False เพื่อให้ ESP32 ดาวน์โหลดได้
    return send_from_directory(UPLOAD_FOLDER, filename, as_attachment=False)

if __name__ == '__main__':
    # รัน Web Server ให้เข้าถึงได้จากทุก IP ในเครือข่าย
    # อย่าใช้ debug=True ใน production
    app.run(host='0.0.0.0', port=8000) 