import socket
import os
import time

# Configuración de Red
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    
    last_frame = None
    
    os.system('clear' if os.name == 'posix' else 'cls')
    print(f"--- INSPECTOR DE PROTOCOLO CRSF ({UDP_IP}:{UDP_PORT}) ---")
    print("Esperando datos del simulador C++...\n")

    try:
        while True:
            data, addr = sock.recvfrom(1024)
            
            # Validar que sea un paquete CRSF (Sync 0xEE)
            if len(data) >= 26 and data[0] == 0xEE:
                # Volver al inicio de la sección de datos
                print("\033[4;0H", end="") 
                
                # 1. Mostrar Frame en HEX
                hex_str = " ".join(f"{b:02X}" for b in data)
                print(f"Frame Completo (HEX):\n{hex_str}\n")
                
                # 2. Desglose estructural
                print("Estructura del Paquete:")
                print(f"  [0x{data[0]:02X}] Sync/Dest (FC)")
                print(f"  [0x{data[1]:02X}] Longitud ({data[1]} bytes)")
                print(f"  [0x{data[2]:02X}] Tipo (RC Channels)")
                
                # 3. Comparador simple (Detectar cambios)
                if last_frame:
                    cambios = ""
                    for i in range(len(data)):
                        if data[i] != last_frame[i]:
                            cambios += f" byte[{i}]"
                    print(f"\033[KÚltimos cambios detectados: {cambios if cambios else 'Ninguno'}")
                
                last_frame = data

    except KeyboardInterrupt:
        print("\n\nInspector cerrado.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()