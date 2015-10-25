from django.shortcuts import render

# Create your views here.

from django.http import HttpResponseRedirect, HttpResponse
from django.core.urlresolvers import reverse

# django uses protection from Cross Site Request Forgery (CSRF)
# therefore you need to make an exception for logdata POST request 
from django.views.decorators.csrf import csrf_exempt

from django.contrib.auth.models import User
from django.utils import timezone
import datetime, sys, logging

from .models import Horse, HorseTracker, HorseData


# ...

@csrf_exempt
def addData(request):

    t_version = request.POST['ver']      # version of software/hardware
    t_imei    = request.POST['IMEI']     # imei (modem) number
    t_imsi    = request.POST['IMSI']     # imsi (simcard) number
    t_email   = request.POST['user']     # users email adress
    data      = request.POST['data']     # logger data
    sum       = int(request.POST['sum']) # bit sum of sent string (checksum)
    
    # Assembley string for bit sum check
    check_string =("ver=" + t_version + "&IMEI=" + t_imei + "&IMSI=" + 
                t_imsi + "&user=" + t_email + "&data=" + data + "&sum=")
    check_sum = ''.join(format(ord(x), 'b') for x in check_string).count('1')

    # split up data string into each log event
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


        logdate = datetime.datetime(year, month, day, hour, min, sec)
        #how to print to cloudware@uwsgi logg
        #print(datum, file=sys.stderr)

        # grab the user from the database reported by the tracker
        try: 
            current_user = User.objects.get(email=t_email)
        except User.DoesNotExist:
            print("DATA:User Does Not Exist", file=sys.stderr)
            return HttpResponse("ERROR")

        # with user object grab the corresponding tracker object
        # if not existing, then register a new tracker
        try:
            current_tracker = HorseTracker.objects.get(IMEI=t_imei, IMSI=t_imsi,
                                                       user=current_user)
            if( current_tracker.version != t_version ):
                current_tracker.version = t_version
                current_tracker.save()

        except HorseTracker.DoesNotExist:
            print("DATA:New tracker found, registering it!", file=sys.stderr)
            current_tracker = HorseTracker(IMEI=t_imei, IMSI=t_imsi, 
                                           user=current_user, version=t_version)
            current_tracker.save()
            print(current_tracker, file=sys.stderr)

        # create a new post of tracker positioning data
        incoming_data = HorseData(tracker=current_tracker, date=logdate,
                     latitude=lat, longitude=lon,
                     pressure=0, humidity=value2, temperature=value3,
                     MPUtemp=value4, 
                     accX=value5, accY=value6, accZ=value7,
                     magX=value8, magY=value9, magZ=value10,
                     batteryVoltage=value1, batteryCharge=value0
        )
        incoming_data.save()
        return HttpResponse("OK")
    return HttpResponse("ERROR")

def index(request):
    
    return HttpResponse("Hello, world. You're at the polls index.")
