from django.shortcuts import render

# Create your views here.

from django.http import HttpResponseRedirect, HttpResponse
from django.core.urlresolvers import reverse

# django uses protection from Cross Site Request Forgery (CSRF)
# therefore you need to make an exception for logdata POST request 
from django.views.decorators.csrf import csrf_exempt

from django.contrib.auth.models import User
from django.utils import timezone
import datetime, pytz, sys, logging

from .models import Horse, HorseTracker, HorseData
from django.views import generic
from django.template import RequestContext, loader
from django.conf import settings
from django.shortcuts import redirect
from django.contrib.auth.decorators import login_required
from django.contrib.auth.decorators import permission_required
# ...

@csrf_exempt
def AddData(request):

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
       
        for i in range(0,length,17):
            #if version == 1:

                value0 = int(data_array[i])     #batt %
                value1 = int(data_array[i+1])   #batt milliVolt
                value2 = float(data_array[i+2]) #BME280 temp / DHT11 temp
                value3 = float(data_array[i+3]) #BME280 hum / DHT11 hum
                value4 = float(data_array[i+4]) #BME280 pressure
                value5 = float(data_array[i+5]) #IR temp
                value6 = float(data_array[i+6]) #MPU temp
                value7 = int(data_array[i+7])   #acx
                value8 = int(data_array[i+8])   #acy
                value9 = int(data_array[i+9])   #acz
                value10 = int(data_array[i+10])   #max
                value11 = int(data_array[i+11])  #may
                value12 = int(data_array[i+12]) #maz

                lat = float(data_array[i+13])
                lon = float(data_array[i+14])

                hour = int(data_array[i+15][0] + data_array[i+15][1])
                min = int(data_array[i+15][2] + data_array[i+15][3])
                sec = int(data_array[i+15][4] + data_array[i+15][5])
 
                day = int(data_array[i+16][0] + data_array[i+16][1])
                month = int(data_array[i+16][2] + data_array[i+16][3])
                year = int('20' + data_array[i+16][4] + data_array[i+16][5])


                logdate = datetime.datetime(year, month, day, hour, min, 
                                        sec,tzinfo=pytz.timezone('UTC'))
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
                         pressure=value4, humidity=value3, temperature=value2,
                         IRtemp=value5, MPUtemp=value6, 
                         accX=value7, accY=value8, accZ=value9,
                         magX=value10, magY=value11, magZ=value12,
                         batteryVoltage=value1, batteryCharge=value0
                )
                incoming_data.save()
        return HttpResponse("OK")
    return HttpResponse("ERROR")


@login_required
@permission_required('data.tracker_user')
def HorseDataDetail(request, pk):
    return HttpResponse("You're looking at log data %s." % pk)

def HorseDataList(request):

    if request.user.is_authenticated():
        log_list = HorseData.objects.order_by('date')
        #template = loader.get_template('data/index.html')
        #context = RequestContext(request, {
        #    'log_list': log_list,
        #}) 
        #return HttpResponse(template.render(context))
        context = {'log_list': log_list}
        return render(request, 'data/index.html', context)
    else:
        return redirect('%s?next=%s' % (settings.LOGIN_URL, request.path))
        #return HttpResponse("Not logged in!")

def index(request):

    return HttpResponse("Hello, world. You're at the polls index.")


class IndexView(generic.ListView):

        model=HorseData
        template_name = 'data/index.html'
        context_object_name = 'log_list'

        def get_queryset(self):
           """Return the last five published questions."""
           return HorseData.objects.order_by('-date')

