from django.contrib import admin

# Register your models here.

from .models import HorseTracker
from .models import Horse
from .models import HorsePositionData

admin.site.register(HorseTracker)
admin.site.register(Horse)
admin.site.register(HorsePositionData)





