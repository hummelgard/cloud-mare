from django.shortcuts import render

# Create your views here.

from django.http import HttpResponseRedirect, HttpResponse
from django.core.urlresolvers import reverse

# django uses protection from Cross Site Request Forgery (CSRF)
# therefore you need to make an exception for logdata POST request 
from django.views.decorators.csrf import csrf_exempt

from django.utils import timezone
import datetime
import sys
from .models import Horse, HorseTracker, HorsePositionData
import logging

# ...

@csrf_exempt
def addData(request):
    version = request.POST['ver']
    imei = request.POST['IMEI']
    imsi = request.POST['IMSI']
    name = request.POST['name']
    data = request.POST['data']
    sum = int(request.POST['sum'])
    check_string =("ver=" + version + "&IMEI=" + imei + "&IMSI=" + 
                imsi + "&name=" + name + "&data=" + data + "&sum=")
    check_sum = ''.join(format(ord(x), 'b') for x in check_string).count('1')
    data_array = data.split('#')
    length = len(data_array) 

    if check_sum == sum:
       
        for i in range(0,length,15):
            #if version == 1:

                value0 = int(data_array[i])     #batt %
                value1 = int(data_array[i+1])   #batt milliVolt
                value2 = float(data_array[i+2]) #DHT11 hum
                value3 = float(data_array[i+3]) #DHT11 temp
                value4 = float(data_array[i+4]) #MPU temp
                value5 = int(data_array[i+5])   #acx
                value6 = int(data_array[i+6])   #acy
                value7 = int(data_array[i+7])   #acz
                value8 = int(data_array[i+8])   #max
                value9 = int(data_array[i+9])   #may
                value10 = int(data_array[i+10]) #maz

                lat = float(data_array[i+11])
                lon = float(data_array[i+12])

                hour = int(data_array[i+13][0] + data_array[i+13][1])
                min = int(data_array[i+13][2] + data_array[i+13][3])
                sec = int(data_array[i+13][4] + data_array[i+13][5])
 
                day = int(data_array[i+14][0] + data_array[i+14][1])
                month = int(data_array[i+14][2] + data_array[i+14][3])
                year = int('20' + data_array[i+14][4] + data_array[i+14][5])

#                time = (data_array[i+13][0] + data_array[i+13][1] + ":" +
#                        data_array[i+13][2] + data_array[i+13][3] + ":" +
#                        data_array[i+13][4] + data_array[i+13][5])

#                date = (data_array[i+14][0] + data_array[i+14][1] + "-" + 
#                        data_array[i+14][2] + data_array[i+14][3] + "-" + 
#                        data_array[i+14][4] + data_array[i+14][5])   
    
    datum=datetime.datetime(year, month, day, hour, min, sec)
    print(datum, file=sys.stderr)

    device=HorseTracker.objects.filter(IMEI=imei).get()

    incoming_data = HorsePositionData(tracker=device, date=datum,
                     latitude=lat, longitude=lon,
                     pressure=0, humidity=value2, temperature=value3,
                     MPUtemp=value4, 
                     accX=value5, accY=value6, accZ=value7,
                     magX=value8, magY=value7, magZ=value8,
                     batteryVoltage=value1, batteryCharge=value0
    )
    incoming_data.save()

    return HttpResponse("Hello, world. You're at the polls index.")

def index(request):
    
    HorseTracker
    return HttpResponse("Hello, world. You're at the polls index.")
