import zmq, json, base64, cv2
import numpy as np

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect('tcp://127.0.0.1:5555')
sock.setsockopt_string(zmq.SUBSCRIBE, '')
sock.setsockopt(zmq.RCVTIMEO, 2000)

print('Waiting for frames... press Q to quit')
while True:
    try:
        msg = json.loads(sock.recv_string())
        if 'wrist' in msg['images']:
            jpg = base64.b64decode(msg['images']['wrist'])
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('wrist', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except zmq.Again:
        print('Timeout - no frame received')
cv2.destroyAllWindows()
