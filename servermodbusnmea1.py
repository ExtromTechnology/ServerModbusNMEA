import paho.mqtt.client as mqtt
import socket
import json
import threading
import time
import struct
import array as arr

# Server ActiveMQ port MQTT
username 	= ""
password 	= ""
hostname 	= "MQTT server address"
port 	  	= <MQTT port>

#topic
topic1 = "220518491100001/ModBus"
topic2 = "220518491100001/LWT"
#topic1 = "210412491100008/ModBus"
#topic2 = "210412491100008/LWT"
topic3 = "210412491100008/NMEA"
#topic3 = "210412491100006/NMEA"

rc = 0;

from pymodbus.server.sync import StartTcpServer, ModbusTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from twisted.internet.task import LoopingCall
from twisted.internet import reactor
import threading
import logging

# --------------------------------------------------------------------------- # 
# import the payload builder
# --------------------------------------------------------------------------- # 

#from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder

UNIT = 0x01
#bind_ip = 'localhost';
bind_ip = '192.168.1.117';
bind_port = 32008;
#bind_port = 5020;

# register map modbus
registros = arr.array('I',[0 for i in range(109)]);

# --------------------------------------------------------------------------- # 
# configure the service logging
# --------------------------------------------------------------------------- # 
import logging
#FORMAT = ('%(asctime)-15s %(threadName)-15s'
#          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
#logging.basicConfig(format=FORMAT)
#log = logging.getLogger()
#log.setLevel(logging.DEBUG)

def main():

    servermqtt = socket.socket(socket.AF_INET, socket.SOCK_STREAM);

    try:
        mqttc = mqtt.Client()
        mqttc.on_message = on_message
        mqttc.on_connect = on_connect
        mqttc.on_publish = on_publish
        mqttc.on_subscribe = on_subscribe
        mqttc.username_pw_set(username, password)
        mqttc.connect(hostname, port)
        #mqttc.subscribe(topic1,0)
        #mqttc.subscribe(topic2,0)
        mqttc.subscribe(topic3,0)
        servermqtt.bind((bind_ip,bind_port));
        servermqtt.listen(6);
        print ('> Listen %s:%d' %(bind_ip,bind_port));
    except:
        print('\nCould not start the server!\n');
        exit();

    thread1 = threading.Thread(target=loopmqtt, args=[mqttc]);
    thread1.start();
    
def loopmqtt(mqttc):    
    while True:
        mqttc.loop_start()
    
def on_connect(client, userdata, flags, rc):
    print("rc: " + str(rc))
    
def insertregister(regi, mens, strr):
    registro.pop(regi);
    registro.insert(regi,int(mens[strr]));

# In the NMEA message, the position gets transmitted as:
# DDMM.MMMMM, where DD denotes the degrees and MM.MMMMM denotes
# the minutes. However, I want to convert this format to the following:
# DD.MMMM. This method converts a transmitted string to the desired format
def formatDegreesMinutes(coordinates, digits):
    
    parts = coordinates.split(".")

    if (len(parts) != 2):
        return coordinates

    if (digits > 3 or digits < 2):
        return coordinates
    
    left = parts[0]
    right = parts[1]
    degrees = str(left[:digits])
    minutes = str(right[:3])

    return degrees + "." + minutes
    
def getPositionData(gps):
    
    #decompose the NMEA
    
    parts = gps.split(",")

    if ((parts[0] == "$GNGGA") or (parts[0] == "$GPGGA")):
        
        #"$GNGGA,182313.000,2257.8956,S,04256.2417,W,1,04,3.2,33.0,M,-6.5,M,,0000*69"
        UTC = formatDegreesMinutes(parts[1], 4)
        z = float(UTC)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[0] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[1] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        latitude = formatDegreesMinutes(parts[2], 4)
        z = float(latitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[2] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[3] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
    
        for x in parts[3]: latitudeE =  ord(x)
        registros[4] = latitudeE;

        longitude = formatDegreesMinutes(parts[4], 4)
        z = float(longitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[5] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[6] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[5]: latitudeE =  ord(x)
        registros[7] = latitudeE;
        
        #"$GNGGA,182313.000,2257.8956,S,04256.2417,W,1,04,3.2,33.0,M,-6.5,M,,0000*69"
        QPS = formatDegreesMinutes(parts[6], 2)
        registros[8] = int(QPS);
        
        NumSatelites = formatDegreesMinutes(parts[7], 2)
        if NumSatelites == "":
            NumSatelites = 0
        registros[9] = int(NumSatelites);
        
        DHP = formatDegreesMinutes(parts[8], 2)
        if DHP == "":
            DHP = 0
        z = float(DHP)
        DHP = struct.unpack("HH", struct.pack("f", z))
        registros[10] = ((DHP[0] & 0x00FF) << 8) | ((DHP[0] & 0xFF00) >> 8);
        registros[11] = ((DHP[1] & 0x00FF) << 8) | ((DHP[1] & 0xFF00) >> 8);

        AntAlt = formatDegreesMinutes(parts[9], 2)
        if AntAlt == "":
            AntAlt = 0
        z = float(AntAlt)
        AntAlt = struct.unpack("HH", struct.pack("f", z))
        registros[12] = ((AntAlt[0] & 0x00FF) << 8) | ((AntAlt[0] & 0xFF00) >> 8);
        registros[13] = ((AntAlt[1] & 0x00FF) << 8) | ((AntAlt[1] & 0xFF00) >> 8);

        for x in parts[10]: Unit =  ord(x)
        registros[14] = Unit;
        
        Geodal = formatDegreesMinutes(parts[11], 2)
        if Geodal == "":
            Geodal = 0
        z = float(Geodal)
        Geodal = struct.unpack("HH", struct.pack("f", z))
        registros[15] = ((Geodal[0] & 0x00FF) << 8) | ((Geodal[0] & 0xFF00) >> 8);
        registros[16] = ((Geodal[1] & 0x00FF) << 8) | ((Geodal[1] & 0xFF00) >> 8);

        for x in parts[12]: Unit =  ord(x)
        registros[17] = Unit;
        
        Diferential = formatDegreesMinutes(parts[13], 2)
        if Diferential == "":
            Diferential = 0
        registros[18] = int(Diferential);
        
        c = parts[14]
        DiferentialID = formatDegreesMinutes(c[0], 2)
        if DiferentialID == "":
            DiferentialID = 0
        if DiferentialID == "*":
            DiferentialID = 0
        registros[19] = int(DiferentialID);
    
    #"$GPHDT,202.1,T*34"
    if ((parts[0] == "$GNHDT") or (parts[0] == "$GPHDT")):

        Heading = formatDegreesMinutes(parts[1], 4)
        z = float(Heading)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[20] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[21] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[2]: degrees =  ord(x)
        registros[22] = degrees;
    
    #"$GPMWD,x.x,T,x.x,M,x.x,N,x.x,M*hh"
    if ((parts[0] == "$GNMWD") or (parts[0] == "$GPMWD")):

        WindDirectionTrue = formatDegreesMinutes(parts[1], 4)
        z = float(WindDirectionTrue)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[23] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[24] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[2]: unit =  ord(x)
        registros[25] = unit;

        WindDirectionMag = formatDegreesMinutes(parts[3], 4)
        z = float(WindDirectionMag)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[26] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[27] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[4]: unit =  ord(x)
        registros[28] = unit;

        WindDirectionKnots = formatDegreesMinutes(parts[5], 4)
        z = float(WindDirectionKnots)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[29] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[30] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[6]: unit =  ord(x)
        registros[31] = unit;

        WindDirectionMeters = formatDegreesMinutes(parts[7], 4)
        z = float(WindDirectionMeters)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[32] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[33] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[8]: unit =  ord(x)
        registros[34] = unit;

    #"$GPMWV,318.0,R,014.8,N,A*3A"
    if ((parts[0] == "$GNMWV") or (parts[0] == "$GPMWV")):

        UTC = formatDegreesMinutes(parts[1], 4)
        z = float(UTC)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[35] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[36] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[2]: unit =  ord(x)
        registros[37] = unit;

        UTC = formatDegreesMinutes(parts[3], 4)
        z = float(UTC)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[38] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[39] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[4]: unit =  ord(x)
        registros[40] = unit;

        for x in parts[5]: status =  ord(x)
        registros[41] = status;
        
    #"$GPZDA,205753.00,13,06,2022,-03,00*48"
    if ((parts[0] == "$GNZDA") or (parts[0] == "$GPZDA")):

        UTC = formatDegreesMinutes(parts[1], 4)
        z = float(UTC)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[42] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[43] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        dia = formatDegreesMinutes(parts[2], 2)
        if dia == "":
            dia = 0
        registros[44] = int(dia);
        
        mes = formatDegreesMinutes(parts[3], 2)
        if mes == "":
            mes = 0
        registros[45] = int(mes);
        
        ano = formatDegreesMinutes(parts[4], 2)
        if ano == "":
            ano = 0
        registros[46] = int(ano);
        
        hora = formatDegreesMinutes(parts[5], 2)
        hora = int(hora)
        if hora == "":
            hora = 0
        if (int(hora) < 0):
            hora = (int(65536) + int(hora))
        registros[47] = hora;

        c = parts[6]
        minuto = formatDegreesMinutes(c[0], 2)
        if minuto == "":
            minuto = 0
        if minuto == "*":
            minuto = 0
        registros[48] = int(minuto);
        
    #"$GPVTG,226.7,T,,,0.2,N,0.4,K,A*47"
    if ((parts[0] == "$GNVTG") or (parts[0] == "$GPVTG")):

        CourseovergroundTrue = formatDegreesMinutes(parts[1], 4)
        if CourseovergroundTrue == "":
            CourseovergroundTrue = 0
        z = float(CourseovergroundTrue)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[49] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[50] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
    
        for x in parts[2]: unit =  ord(x)
        registros[51] = unit;

        CourseovergroundMag = formatDegreesMinutes(parts[3], 4)
        if CourseovergroundMag == "":
            CourseovergroundMag = 0
        z = float(CourseovergroundMag)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[52] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[53] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[4]: unit =  ord(x)
        registros[54] = unit;
        
        CourseovergroundKnots = formatDegreesMinutes(parts[5], 4)
        if CourseovergroundKnots == "":
            CourseovergroundKnots = 0
        z = float(CourseovergroundKnots)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[55] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[56] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[6]: unit =  ord(x)
        registros[57] = unit;

        CourseovergroundKm = formatDegreesMinutes(parts[7], 4)
        if CourseovergroundKm == "":
            CourseovergroundKm = 0
        z = float(CourseovergroundKm)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[58] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[59] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[8]: unit =  ord(x)
        registros[60] = unit;

        c = parts[9]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[61] = mode;
        
    #$GPDBT,1330.5,f,0405.5,M,0221.6,F*2E
    if ((parts[0] == "$GNDBT") or (parts[0] == "$GPDBT")):

        Profundidade  = formatDegreesMinutes(parts[1], 4)
        if Profundidade == "":
            Profundidade = 0
        z = float(Profundidade )
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[62] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[63] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[2]: unit =  ord(x)
        registros[64] = unit;

        Profundidade  = formatDegreesMinutes(parts[3], 4)
        if Profundidade == "":
            Profundidade = 0
        z = float(Profundidade )
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[65] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[66] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[4]: unit =  ord(x)
        registros[67] = unit;

        Profundidade  = formatDegreesMinutes(parts[5], 4)
        if Profundidade == "":
            Profundidade = 0
        z = float(Profundidade )
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[68] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[69] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[6]: unit =  ord(x)
        registros[70] = unit;
        
    #"$GPVBW,0.0,,A,-0.8,0.0,A,,,,*5F"
    if ((parts[0] == "$GNVBW") or (parts[0] == "$GPVBW")):

        Velocidade  = formatDegreesMinutes(parts[1], 4)
        if Velocidade == "":
            Velocidade = 0
        z = float(Velocidade)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[71] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[72] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        Velocidade  = formatDegreesMinutes(parts[2], 4)
        if Velocidade == "":
            Velocidade = 0
        z = float(Velocidade)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[73] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[74] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        c = parts[3]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[75] = mode;
        
        Velocidade  = formatDegreesMinutes(parts[4], 4)
        if Velocidade == "":
            Velocidade = 0
        z = float(Velocidade)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[76] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[77] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        Velocidade  = formatDegreesMinutes(parts[5], 4)
        if Velocidade == "":
            Velocidade = 0
        z = float(Velocidade)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[78] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[79] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        c = parts[6]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[80] = mode;
        
    #$GNGGA,134226.000,2257.8975,S,04256.2151,W,1,05,2.1,-19.5,M,-6.5,M,,0000*46
    if ((parts[0] == "$GNGLL") or (parts[0] == "$GPGLL")):
        
        latitude  = formatDegreesMinutes(parts[1], 4)
        if latitude == "":
            latitude = 0
        z = float(latitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[81] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[82] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        c = parts[6]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[83] = mode;
        
        latitude  = formatDegreesMinutes(parts[1], 4)
        if latitude == "":
            latitude = 0
        z = float(latitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[84] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[85] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        c = parts[6]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[86] = mode;
        
        latitude  = formatDegreesMinutes(parts[1], 4)
        if latitude == "":
            latitude = 0
        z = float(latitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[87] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[88] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        c = parts[6]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[89] = mode;
        
        c = parts[6]
        for x in c[0]: mode =  ord(x)
        if mode == "":
            mode = 0
        if mode == "*":
            mode = 0
        registros[90] = mode;
        
    #"$GNRMC,200527.000,A,2257.9095,S,04256.1991,W,000.0,314.4,130622,,,A*79"
    if ((parts[0] == "$GNRMC") or (parts[0] == "$GPRMC")):
        
        UTC = formatDegreesMinutes(parts[1], 4)
        z = float(UTC)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[91] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[92] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);
        
        for x in parts[2]: latitudeE =  ord(x)
        registros[93] = latitudeE;

        latitude = formatDegreesMinutes(parts[3], 4)
        z = float(latitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[94] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[95] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[4]: latitudeE =  ord(x)
        registros[96] = latitudeE;

        longitude = formatDegreesMinutes(parts[5], 4)
        z = float(longitude)
        wordfloat = struct.unpack("HH", struct.pack("f", z))
        registros[97] = ((wordfloat[0] & 0x00FF) << 8) | ((wordfloat[0] & 0xFF00) >> 8);
        registros[98] = ((wordfloat[1] & 0x00FF) << 8) | ((wordfloat[1] & 0xFF00) >> 8);

        for x in parts[6]: latitudeE =  ord(x)
        registros[99] = latitudeE;
        
        Speedover = formatDegreesMinutes(parts[7], 2)
        z = float(Speedover)
        Speedover = struct.unpack("HH", struct.pack("f", z))
        registros[100] = ((Speedover[0] & 0x00FF) << 8) | ((Speedover[0] & 0xFF00) >> 8);
        registros[101] = ((Speedover[1] & 0x00FF) << 8) | ((Speedover[1] & 0xFF00) >> 8);

        CourseMadeGood = formatDegreesMinutes(parts[8], 2)
        z = float(CourseMadeGood)
        CourseMadeGood = struct.unpack("HH", struct.pack("f", z))
        registros[102] = ((CourseMadeGood[0] & 0x00FF) << 8) | ((CourseMadeGood[0] & 0xFF00) >> 8);
        registros[103] = ((CourseMadeGood[1] & 0x00FF) << 8) | ((CourseMadeGood[1] & 0xFF00) >> 8);

        Dateoffix = formatDegreesMinutes(parts[9], 2)
        z = float(Dateoffix)
        Dateoffix = struct.unpack("HH", struct.pack("f", z))
        registros[104] = ((Dateoffix[0] & 0x00FF) << 8) | ((Dateoffix[0] & 0xFF00) >> 8);
        registros[105] = ((Dateoffix[1] & 0x00FF) << 8) | ((Dateoffix[1] & 0xFF00) >> 8);

        Magneticvariation = formatDegreesMinutes(parts[10], 4)
        z = float(Magneticvariation)
        Magneticvariation = struct.unpack("HH", struct.pack("f", z))
        registros[106] = ((Magneticvariation[0] & 0x00FF) << 8) | ((Magneticvariation[0] & 0xFF00) >> 8);
        registros[107] = ((Magneticvariation[1] & 0x00FF) << 8) | ((Magneticvariation[1] & 0xFF00) >> 8);

        for x in parts[11]: MagneticvariationE =  ord(x)
        registros[108] = MagneticvariationE;
        
def on_message(client, obj, msg):

    if (msg.topic == topic3):
        print("NMEA: "+str(msg.payload))
        mens = json.loads(str(msg.payload))
        getPositionData(mens["Message"])

def on_publish(client, obj, mid):
    print("mid: " + str(mid))

def on_subscribe(client, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(client, obj, level, string):
    print(string)

def run_server():

    builder = BinaryPayloadBuilder()
    
    builder.add_16bit_int(0x0001)
    builder.add_16bit_int(0x0002)
    builder.add_16bit_int(0x0003)
    builder.add_16bit_int(0x0004)
    builder.add_16bit_int(0x0005)
    builder.add_16bit_int(0x0006)
    builder.add_16bit_int(0x0007)
    builder.add_16bit_int(0x0008)
    builder.add_16bit_int(0x0009)
    builder.add_16bit_int(0x000A)
    builder.add_16bit_int(0x000B)
    builder.add_16bit_int(0x000C)
    builder.add_16bit_int(0x000D)
    builder.add_string('abcdefgh')
    builder.add_bits([0, 1, 0, 1, 1, 0, 1, 0])
    builder.add_8bit_int(-0x01)
    builder.add_8bit_uint(0x12)
    builder.add_8bit_uint(0x13)
    builder.add_32bit_float(22.34)
    builder.add_32bit_float(-22.34)

    #block = ModbusSequentialDataBlock(1, builder.to_registers())
    block = ModbusSequentialDataBlock(0, [0]*0xff)
    block1 = ModbusSequentialDataBlock(0, [0]*0xff)
    
    slaves1 = {0x01: ModbusSlaveContext(di=block,  co=block,  hr=block,  ir=block),
               0x02: ModbusSlaveContext(di=block1, co=block1, hr=block1, ir=block1)
              }

    context = ModbusServerContext(slaves=slaves1, single=False)
    
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Extrom Technology'
    identity.ProductCode = 'MS4911'
    identity.VendorUrl = 'https://www.extromtechnology.com'
    identity.ProductName = 'Modbus/NMEA Server'
    identity.ModelName = 'Modbus Server'
    identity.MajorMinorRevision = '1.0'

    server = ModbusTcpServer(context,identity=identity,address=(bind_ip, bind_port))

    thread_ = threading.Thread(target=server.serve_forever)
    thread_.start()
    
    interval = 0.2
    loop = LoopingCall(f=update_values, a=server)
    loop.start(interval, now=True)
    reactor.run()
    
def update_reg(context_, addrs):
    wfuncode = 3
    address = addrs
    values = [registros[addrs]]
    context_.setValues(wfuncode, address, values)

def update_values(a):
    rfuncode = 3
    wfuncode = 3
    # select slave to update
    slave_id = 0x01
    context_ = a.context[slave_id]
    
    #values = context_.getValues(rfuncode, address, count=1) 
    #values = [val+1 for val in values]

    for w in range(109):
        update_reg(context_,w)

    #address = 0x00
    #values = [registro[0]];
    #context_.setValues(wfuncode, address, values)
    #address = 0x01
    #values = [registro[1]];
    #context_.setValues(wfuncode, address, values)

main()
run_server()

