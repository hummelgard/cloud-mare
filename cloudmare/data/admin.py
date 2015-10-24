from django.utils.translation import ugettext_lazy as _
from django.contrib import admin

# Register your models here.

from .models import HorseTracker
from .models import Horse
from .models import HorsePositionData

class HorsePositionDataAdmin(admin.ModelAdmin):
    #
    list_display = ('date', 'latitude', 'longitude', 'temperature', 'batteryCharge')
    list_filter = ['date']
    search_fields = ['date']
    fieldsets = [
        (None,              {'fields': ['tracker','date']}),
        (_('Position Data'),   {'fields': ['latitude','longitude']}),
        (_('Weather Data'),    {'fields': ['pressure','humidity','temperature']}),
        (_('MPU Data'),        {'fields': 
['MPUtemp','accX','accY','accZ','magX','magY','magZ'], 'classes': ['collapse']}),
        (_('Tracker Status'),    {'fields': ['batteryVoltage','batteryCharge'], 'classes': 
['collapse']}),

        ]


admin.site.register(HorseTracker)
admin.site.register(Horse)
admin.site.register(HorsePositionData, HorsePositionDataAdmin)





