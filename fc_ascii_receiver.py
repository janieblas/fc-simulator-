import socket
import time
import os

# Configuración de Red
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

def decode_crsf_channels(payload):
    ch = [0] * 8
    ch[0] = (payload[0] | (payload[1] << 8)) & 0x07FF
    ch[1] = ((payload[1] >> 3) | (payload[2] << 5)) & 0x07FF
    ch[2] = ((payload[2] >> 6) | (payload[3] << 2) | (payload[4] << 10)) & 0x07FF
    ch[3] = ((payload[4] >> 1) | (payload[5] << 7)) & 0x07FF
    ch[4] = ((payload[5] >> 4) | (payload[6] << 4)) & 0x07FF
    ch[5] = ((payload[6] >> 7) | (payload[7] << 1) | (payload[8] << 9)) & 0x07FF
    ch[6] = ((payload[8] >> 2) | (payload[9] << 6)) & 0x07FF
    ch[7] = ((payload[9] >> 5) | (payload[10] << 3)) & 0x07FF
    return ch

def draw_ui(channels):
    SIZE = 13 
    HALF = SIZE // 2
    
    # Mapeo de coordenadas
    def get_pos(val_h, val_v):
        x = int((val_h + 1) * HALF + 0.5)
        y = int((1 - val_v) * HALF + 0.5)
        return max(0, min(SIZE-1, x)), max(0, min(SIZE-1, y))

    lx, ly = get_pos(channels[3], channels[2])
    rx, ry = get_pos(channels[0], channels[1])

    # Generar rejillas
    def make_grid(sx, sy):
        grid = [["  " for _ in range(SIZE)] for _ in range(SIZE)]
        grid[HALF][HALF] = "++"
        grid[sy][sx] = "[]"
        return grid

    g_l, g_r = make_grid(lx, ly), make_grid(rx, ry)

    # Construir el bloque de texto
    output = []
    output.append("\033[H") # Volver al origen
    output.append("\n      STICK IZQUIERDO (Y/T)                STICK DERECHO (R/P)")
    
    border = "+" + "-"*(SIZE*2) + "+"
    output.append(f"    {border}      {border}")

    for y in range(SIZE):
        line_l = "".join(g_l[y])
        line_r = "".join(g_r[y])
        output.append(f"    |{line_l}|      |{line_r}|")

    output.append(f"    {border}      {border}")
    output.append(f"     T:{channels[2]:>5.2f} Y:{channels[3]:>5.2f}               P:{channels[1]:>5.2f} R:{channels[0]:>5.2f}")
    
    return "\n".join(output)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    os.system('clear' if os.name == 'posix' else 'cls')
    
    channels = [0.0] * 8

    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
                if len(data) >= 26 and data[0] == 0xEE:
                    payload = data[3:25]
                    raw_ch = decode_crsf_channels(payload)
                    channels = [(c - 992) / 819.5 for c in raw_ch]
            except BlockingIOError:
                pass

            # Renderizar solo los cuadros y valores
            print(draw_ui(channels), end="", flush=True)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nSaliendo del simulador...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()