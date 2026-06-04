import cv2
import numpy as np
import time
import argparse
import sys

try:
    import serial
except Exception:
    serial = None

INDICE_VIDEO = 0
PIXEL_ANCHO = 40
PIXEL_ALTO = 30
TOL_HSV = (10, 60, 60)
BAUD_SERIAL = 9600
PUERTO_SERIAL = None            
INTERVALO_ENVIO = 0.12         

NOMBRE_CAM_OBJETIVO = 'UVC Camera'

RANGOS_SERVOS = {
    'base': (0, 270),      
    'hombro': (0, 90),    
    'codo': (0, 40),      
    'pinza': (0, 30)      
}

POS_REPOSO = {
    'base': 135,        
    'hombro': 45,      
    'codo': 20,        
    'pinza': 30        
}

POS_AGARRE = {
    'pinza': 20         
}

POS_LEVANTA = {
    'hombro': 90,      
    'codo': 40         
}

def pedir_puerto_serie(default=None):
    if serial is None:
        return None
    puertos = []
    try:
        from serial.tools import list_ports
        puertos = [p.device for p in list_ports.comports()]
    except Exception:
        puertos = []
    if default and default in puertos:
        usar = default
    else:
        usar = None
    try:
        choice = input().strip()
    except Exception:
        choice = ''
    if choice.lower() in ('n', 'none'):
        return None
    if choice == '' and usar:
        return usar
    return choice or usar

def mapear_rango(val, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    v = float(val - in_min) / float(in_max - in_min)
    return out_min + v * (out_max - out_min)

def limitar(v, a, b):
    return max(min(v, b), a)

def parsear_args():
    p = argparse.ArgumentParser()
    p.add_argument('--port', '-p', default=PUERTO_SERIAL)
    p.add_argument('--video', '-v', default=INDICE_VIDEO, type=int)
    return p.parse_args()

def main():
    args = parsear_args()
    referencia_servos = None  
    puerto_arduino = None
    conexion_serie = None
    if serial is not None:
        try:
            from serial.tools import list_ports
            puertos_disp = [p.device for p in list_ports.comports()]
            puertos_intento = puertos_disp.copy()
            if 'COM7' in puertos_intento:
                puertos_intento.remove('COM7')
                puertos_intento = ['COM7'] + puertos_intento
            
            if puertos_intento:
                for puerto_intento in puertos_intento:
                    try:
                        conexion_serie = serial.Serial(puerto_intento, BAUD_SERIAL, timeout=1)
                        time.sleep(0.5)
                        puerto_arduino = puerto_intento
                        break
                    except Exception:
                        try:
                            if conexion_serie:
                                conexion_serie.close()
                        except Exception:
                            pass
                        conexion_serie = None
                
                if conexion_serie and puerto_arduino:
                    referencia_servos = dict(POS_REPOSO)
                    try:
                        cmd = f"S1:{referencia_servos['base']},S2:{referencia_servos['hombro']},S3:{referencia_servos['codo']},S4:{referencia_servos['pinza']}\n"
                        conexion_serie.write(cmd.encode())
                    except Exception:
                        pass
        except Exception:
            conexion_serie = None
    
    if conexion_serie is None and puerto_arduino is not None and serial is not None:
        try:
            conexion_serie = serial.Serial(puerto_arduino, BAUD_SERIAL, timeout=1)
            time.sleep(2)
            referencia_servos = dict(POS_REPOSO)
        except Exception:
            conexion_serie = None
    
    if referencia_servos is None:
        referencia_servos = dict(POS_REPOSO)

    def encontrar_indice_camara(max_index=6):
        backend = cv2.CAP_DSHOW if sys.platform.startswith('win') else cv2.CAP_ANY
        for idx in range(max_index):
            cap_try = cv2.VideoCapture(idx, backend) if backend != cv2.CAP_ANY else cv2.VideoCapture(idx)
            if cap_try.isOpened():
                try:
                    cap_try.release()
                except Exception:
                    pass
                return idx
            try:
                cap_try.release()
            except Exception:
                pass
        return None

    try:
        from serial.tools import list_ports
        puertos = [p.device for p in list_ports.comports()]
    except Exception:
        pass

    captura = None
    if NOMBRE_CAM_OBJETIVO and sys.platform.startswith('win'):
        try:
            from pygrabber.dshow_graph import FilterGraph
            fg = FilterGraph()
            nombres_disp = fg.get_input_devices()
            import unicodedata
            def normalizar(s):
                s = str(s).lower()
                s = unicodedata.normalize('NFKD', s)
                return ''.join(ch for ch in s if not unicodedata.combining(ch))
            objetivo_norm = normalizar(NOMBRE_CAM_OBJETIVO)
            nombre_encontrado = None
            for nombre in nombres_disp:
                if objetivo_norm in normalizar(nombre):
                    nombre_encontrado = nombre
                    break
            if nombre_encontrado:
                try:
                    idx = nombres_disp.index(nombre_encontrado)
                    try:
                        captura_intento = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                        time.sleep(0.5)
                        if captura_intento is not None and captura_intento.isOpened():
                            captura = captura_intento
                        else:
                            try:
                                captura_intento.release()
                            except Exception:
                                pass
                    except Exception:
                        pass
                except Exception:
                    pass

                if captura is None:
                    try:
                        captura_intento = cv2.VideoCapture(f'video={nombre_encontrado}', cv2.CAP_DSHOW)
                        time.sleep(0.5)
                        if captura_intento is not None and captura_intento.isOpened():
                            captura = captura_intento
                        else:
                            try:
                                captura_intento.release()
                            except Exception:
                                pass
                    except Exception:
                        pass
        except Exception:
            pass

    if captura is None:
        indice_camara = encontrar_indice_camara(6)
        if indice_camara is None:
            return
        captura = cv2.VideoCapture(indice_camara, cv2.CAP_DSHOW) if sys.platform.startswith('win') else cv2.VideoCapture(indice_camara)
        if not captura.isOpened():
            return

    seleccionado = False
    hsv_objetivo = None
    ultimo_envio = 0
    
    estado_agarre = 'esperando'
    tiempo_agarre = 0
    angulos_objetivo = None  

    def al_hacer_mouse(event, x, y, flags, param):
        nonlocal seleccionado, hsv_objetivo
        if event == cv2.EVENT_LBUTTONDOWN:
            pix_img, hsv_completa = param
            h_pix, w_pix = pix_img.shape[:2]
            px = int(x * w_pix / ancho_disp)
            py = int(y * h_pix / alto_disp)
            px = limitar(px, 0, w_pix-1)
            py = limitar(py, 0, h_pix-1)
            bgr = pix_img[py, px]
            seleccionado = True
            hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0,0]
            hsv_objetivo = hsv

    cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Pixelated', cv2.WINDOW_NORMAL)

    global ancho_disp, alto_disp
    ancho_disp, alto_disp = 640, 480

    cv2.setMouseCallback('Pixelated', al_hacer_mouse, None)

    while True:
        ret, frame = captura.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h_full, w_full = frame.shape[:2]
        pequeña = cv2.resize(frame, (PIXEL_ANCHO, PIXEL_ALTO), interpolation=cv2.INTER_LINEAR)
        pixelado = cv2.resize(pequeña, (ancho_disp, alto_disp), interpolation=cv2.INTER_NEAREST)
        hsv_completa = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mascara = None
        centroide = None
        angulos = None

        if estado_agarre == 'esperando':
            angulos = (referencia_servos['base'], referencia_servos['hombro'], referencia_servos['codo'], referencia_servos['pinza'])
            
            if seleccionado and hsv_objetivo is not None:
                th, ts, tv = int(hsv_objetivo[0]), int(hsv_objetivo[1]), int(hsv_objetivo[2])
                h_tol, s_tol, v_tol = TOL_HSV
                lower = np.array([max(th - h_tol, 0), max(ts - s_tol, 0), max(tv - v_tol, 0)])
                upper = np.array([min(th + h_tol, 179), min(ts + s_tol, 255), min(tv + v_tol, 255)])
                mascara = cv2.inRange(hsv_completa, lower, upper)
                mascara = cv2.medianBlur(mascara, 5)
                kernel = np.ones((5,5), np.uint8)
                mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
                
                M = cv2.moments(mascara)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroide = (cx, cy)
                    
                    bmin, bmax = RANGOS_SERVOS['base']
                    ang_base = mapear_rango(cx, 0, w_full, bmin, bmax)
                    
                    smin, smax = RANGOS_SERVOS['hombro']
                    ang_hombro = mapear_rango(cy, h_full, 0, smax, smin)
                    
                    emin, emax = RANGOS_SERVOS['codo']
                    area = M['m00']
                    area_norm = area / (w_full * h_full)
                    ang_codo = mapear_rango(area_norm, 0.0, 0.05, emax, emin)
                    
                    pmin, pmax = RANGOS_SERVOS['pinza']
                    ang_pinza = pmax  
                    
                    angulos_objetivo = (
                        int(limitar(ang_base, bmin, bmax)),
                        int(limitar(ang_hombro, smin, smax)),
                        int(limitar(ang_codo, emin, emax)),
                        ang_pinza
                    )
                    
                    estado_agarre = 'moviendo'
                    tiempo_agarre = time.time()
        
        elif estado_agarre == 'moviendo':
            angulos = angulos_objetivo
            if time.time() - tiempo_agarre > 1.0:
                estado_agarre = 'agarrando'
                tiempo_agarre = time.time()
        
        elif estado_agarre == 'agarrando':
            ang_base, ang_hombro, ang_codo, _ = angulos_objetivo
            pmin, pmax = RANGOS_SERVOS['pinza']
            ang_pinza = pmin  
            angulos = (ang_base, ang_hombro, ang_codo, ang_pinza)
            if time.time() - tiempo_agarre > 0.5:
                estado_agarre = 'levantando'
                tiempo_agarre = time.time()
        
        elif estado_agarre == 'levantando':
            ang_base = angulos_objetivo[0]
            ang_hombro = POS_LEVANTA['hombro']
            ang_codo = POS_LEVANTA['codo']
            pmin, pmax = RANGOS_SERVOS['pinza']
            ang_pinza = pmin  
            angulos = (ang_base, ang_hombro, ang_codo, ang_pinza)
            if time.time() - tiempo_agarre > 1.0:
                estado_agarre = 'esperando_sustentacion'
                tiempo_agarre = time.time()
        
        elif estado_agarre == 'esperando_sustentacion':
            ang_base = angulos_objetivo[0]
            ang_hombro = POS_LEVANTA['hombro']
            ang_codo = POS_LEVANTA['codo']
            pmin, pmax = RANGOS_SERVOS['pinza']
            ang_pinza = pmin  
            angulos = (ang_base, ang_hombro, ang_codo, ang_pinza)
            if time.time() - tiempo_agarre > 7.0:
                estado_agarre = 'soltando'
                tiempo_agarre = time.time()
        
        elif estado_agarre == 'soltando':
            ang_base = angulos_objetivo[0]
            ang_hombro = POS_LEVANTA['hombro']
            ang_codo = POS_LEVANTA['codo']
            pmin, pmax = RANGOS_SERVOS['pinza']
            ang_pinza = pmax  
            angulos = (ang_base, ang_hombro, ang_codo, ang_pinza)
            if time.time() - tiempo_agarre > 0.5:
                estado_agarre = 'volviendo'
                tiempo_agarre = time.time()
        
        elif estado_agarre == 'volviendo':
            angulos = (referencia_servos['base'], referencia_servos['hombro'], referencia_servos['codo'], referencia_servos['pinza'])
            if time.time() - tiempo_agarre > 1.0:
                estado_agarre = 'esperando'
                tiempo_agarre = time.time()
                seleccionado = False
                hsv_objetivo = None
                angulos_objetivo = None
                if conexion_serie:
                    try:
                        cmd = f"S1:{referencia_servos['base']},S2:{referencia_servos['hombro']},S3:{referencia_servos['codo']},S4:{referencia_servos['pinza']}\n"
                        conexion_serie.write(cmd.encode())
                    except Exception:
                        pass
        
        elif estado_agarre == 'pausa':
            angulos = (POS_REPOSO['base'], POS_REPOSO['hombro'], POS_REPOSO['codo'], POS_REPOSO['pinza'])
            centroide = None
            mascara = None

        disp = frame.copy()
        if centroide is not None:
            cv2.circle(disp, centroide, 8, (0,255,0), -1)
        
        color_estado = (0,255,0)
        if estado_agarre in ['agarrando', 'levantando', 'esperando_sustentacion', 'soltando']:
            color_estado = (0,165,255)  
        elif estado_agarre == 'pausa':
            color_estado = (0,0,255)  
        
        cv2.putText(disp, f"Estado: {estado_agarre}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_estado, 2)
        
        if angulos:
            cv2.putText(disp, f"Angles: S1={angulos[0]}, S2={angulos[1]}, S3={angulos[2]}, S4={angulos[3]}", 
                       (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
        
        if estado_agarre == 'esperando':
            cv2.putText(disp, "Click en 'Pixelated' para seleccionar objeto", (10,100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        elif estado_agarre == 'pausa':
            cv2.putText(disp, ">>> CICLO COMPLETADO <<<", (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
            cv2.putText(disp, "Presiona 'r' para nuevo ciclo o 'c' para nuevo color", (10,140), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1)
        elif estado_agarre == 'esperando_sustentacion':
            tiempo_restante = max(0, 7.0 - (time.time() - tiempo_agarre))
            cv2.putText(disp, f"Sosteniendo... {tiempo_restante:.1f}s", (10,100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,165,0), 2)

        cv2.imshow('Video', disp)
        cv2.setMouseCallback('Pixelated', al_hacer_mouse, (pixelado, hsv_completa))
        cv2.imshow('Pixelated', pixelado)
        if mascara is not None:
            cv2.imshow('Mask', mascara)

        now = time.time()
        if angulos and (now - ultimo_envio) > INTERVALO_ENVIO:
            ultimo_envio = now
            s1, s2, s3, s4 = angulos
            cmd = f"S1:{s1},S2:{s2},S3:{s3},S4:{s4}\n"
            if conexion_serie:
                try:
                    conexion_serie.write(cmd.encode())
                except Exception:
                    pass

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('x'):
            break
        if key == ord('c'):
            seleccionado = False
            hsv_objetivo = None
            estado_agarre = 'reposo'
        if key == ord('r'):
            estado_agarre = 'reposo'
            angulos = (POS_REPOSO['base'], POS_REPOSO['hombro'], POS_REPOSO['codo'], POS_REPOSO['pinza'])

    captura.release()
    if conexion_serie:
        conexion_serie.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

```

---

### Código de Arduino (Firmware del Brazo Robot)

Este programa para Arduino procesa las cadenas provenientes del puerto serie con el formato `S1:VALOR,S2:VALOR,S3:VALOR,S4:VALOR` enviadas por el software de visión artificial para manipular los 4 servos conectados.

```cpp
#include <Servo.h>

Servo servoBase;
Servo servoHombro;
Servo servoCodo;
Servo servoPinza;

const int PIN_BASE = 9;
const int PIN_HOMBRO = 10;
const int PIN_CODO = 11;
const int PIN_PINZA = 12;

void setup() {
  Serial.begin(9600);
  
  servoBase.attach(PIN_BASE);
  servoHombro.attach(PIN_HOMBRO);
  servoCodo.attach(PIN_CODO);
  servoPinza.attach(PIN_PINZA);
  
  servoBase.write(135);
  servoHombro.write(45);
  servoCodo.write(20);
  servoPinza.write(30);
}

void loop() {
  if (Serial.available() > 0) {
    String cadena = Serial.readStringUntil('\n');
    cadena.trim();
    
    if (cadena.length() > 0) {
      int idx1 = cadena.indexOf("S1:");
      int idx2 = cadena.indexOf("S2:");
      int idx3 = cadena.indexOf("S3:");
      int idx4 = cadena.indexOf("S4:");
      
      if (idx1 != -1 && idx2 != -1 && idx3 != -1 && idx4 != -1) {
        int anguloBase = cadena.substring(idx1 + 3, cadena.indexOf(',', idx1)).toInt();
        int anguloHombro = cadena.substring(idx2 + 3, cadena.indexOf(',', idx2)).toInt();
        int anguloCodo = cadena.substring(idx3 + 3, cadena.indexOf(',', idx3)).toInt();
        int anguloPinza = cadena.substring(idx4 + 3).toInt();
        
        servoBase.write(anguloBase);
        servoHombro.write(anguloHombro);
        servoCodo.write(anguloCodo);
        servoPinza.write(anguloPinza);
      }
    }
  }
}
