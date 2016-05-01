from django import forms
from django.http import HttpResponseRedirect
from django.contrib.auth.models import User
from mezzanine.pages.page_processors import processor_for
from .models import Stable, HorseTracker, Horse



#@processor_for("data/horsetrackers")
#def horsetrackers_slug_processor(request, page):
#    current_user = request.user
#    horsetrackers = HorseTracker.objects.filter(user=current_user)
#
#    return {"horsetrackers": horsetrackers}

#@processor_for("data/horses")
#def horses_slug_processor(request, page, pk):
#    current_user = request.user
#    horses = Horse.objects.filter(user=current_user)
#    
#    return {"horses": horses}


