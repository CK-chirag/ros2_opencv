import argparse, socket, threading

def client_thread(conn, addr, logfile=None):
    print(f'[+] Connection from {addr}')
    buffer = ''
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode('utf-8', errors='ignore')
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if line.strip():
                    print(f'[{addr}] {line}')
                    if logfile:
                        logfile.write(line + '\n')
                        logfile.flush()
    finally:
        conn.close()
        print(f'[-] Disconnected {addr}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--out', default=None, help='Optional file to save messages')
    args = parser.parse_args()

    logfile = open(args.out, 'a') if args.out else None

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((args.host, args.port))
    s.listen(5)
    print(f'[+] Listening on {args.host}:{args.port}')
    try:
        while True:
            conn, addr = s.accept()
            t = threading.Thread(target=client_thread, args=(conn, addr, logfile), daemon=True)
            t.start()
    finally:
        s.close()
        if logfile:
            logfile.close()

if __name__ == '__main__':
    main()

